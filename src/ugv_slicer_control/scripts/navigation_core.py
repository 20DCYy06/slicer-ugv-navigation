#!/usr/bin/env python3
"""无 ROS 副作用的路点校验、原子存储和圆弧/栅格几何运算。"""
import fcntl
import math
import os
from pathlib import Path
import tempfile
from datetime import datetime, timezone
import yaml


def finite(value, name):
    if isinstance(value, bool):
        raise ValueError(name + ' must be numeric')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(name + ' must be finite')
    return result


def planner_limits(source, plugin, planner):
    """Constrain Guard by the selected planner; never borrow another plugin's caps."""
    if plugin not in ('teb_local_planner/TebLocalPlannerROS',
                      'dwa_local_planner/DWAPlannerROS'):
        raise ValueError('Unsupported local planner: ' + str(plugin))
    limits = dict(source)
    keys = (('max_forward', 'max_vel_x'), ('max_angular', 'max_vel_theta'),
            ('accel_linear', 'acc_lim_x'), ('accel_angular', 'acc_lim_theta'))
    if plugin == 'teb_local_planner/TebLocalPlannerROS':
        keys += (('max_reverse', 'max_vel_x_backwards'),)
    for key, planner_key in keys:
        values = (finite(source[key], key), finite(planner[planner_key], planner_key))
        if any(value < 0 if key == 'max_reverse' else value <= 0 for value in values):
            raise ValueError('Invalid source/planner velocity or acceleration limit: ' + key)
        limits[key] = min(values)
    reverse = finite(source['max_reverse'], 'max_reverse')
    if reverse < 0:
        raise ValueError('max_reverse must be nonnegative')
    if plugin == 'dwa_local_planner/DWAPlannerROS':
        if finite(planner['min_vel_x'], 'min_vel_x') != 0:
            raise ValueError('DWA min_vel_x must be zero for forward-only navigation')
        for key, planner_key in (('max_forward', 'max_vel_trans'),
                                 ('accel_linear', 'acc_lim_trans')):
            value = finite(planner[planner_key], planner_key)
            if value <= 0:
                raise ValueError(planner_key + ' must be positive')
            limits[key] = min(limits[key], value)
        limits['max_reverse'] = 0.
    return limits


def validate_waypoints(data):
    if not isinstance(data, dict) or not isinstance(data.get('frame_id'), str) or not data['frame_id']:
        raise ValueError('waypoints YAML requires frame_id')
    points = data.get('waypoints')
    # 人工清空为 `waypoints:` 时 YAML 返回 None，安全视为零个路点。
    # 只兼容显式空字段；拼写错误/非法字符串仍拒绝，避免覆盖损坏数据。
    if 'waypoints' in data and points is None:
        points = data['waypoints'] = []
    if not isinstance(points, list):
        raise ValueError('waypoints must be a list')
    names = set()
    for index, point in enumerate(points, 1):
        if not isinstance(point, dict):
            raise ValueError('waypoint %d must be a mapping' % index)
        for key in ('x', 'y'):
            point[key] = finite(point[key], key)
        # 巡航只以 x/y 到点。兼容旧录点文件中的 yaw，也允许手写路点省略它。
        if 'yaw' in point:
            point['yaw'] = finite(point['yaw'], 'yaw')
        point['z'] = finite(point.get('z', 0), 'z')
        for key in ('wait_s', 'xy_tolerance', 'yaw_tolerance'):
            if key in point:
                point[key] = finite(point[key], key)
                if point[key] < 0:
                    raise ValueError(key + ' must be nonnegative')
        name = point.get('name', 'point_%03d' % index)
        if not isinstance(name, str) or not name:
            raise ValueError('waypoint name must be a nonempty string')
        if name in names:
            raise ValueError('Duplicate waypoint name: ' + name)
        names.add(name)
    return data


def load_waypoints(path):
    return validate_waypoints(yaml.safe_load(Path(path).read_text(encoding='utf-8')))


def append_waypoint(path, frame, point):
    """跨回调/进程加锁；完整写临时文件后原子替换，保留已有点和其它字段。"""
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(str(path) + '.lock', 'a', encoding='utf-8') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        data = yaml.safe_load(path.read_text(encoding='utf-8')) if path.exists() else {'frame_id': frame, 'waypoints': []}
        if isinstance(data, dict) and isinstance(data.get('waypoints'), list):
            # 人工留下的单独 `-` 是 None，不是路点。仅移除这类空占位，绝不吞掉错误坐标。
            points = data['waypoints']
            if any(p is None for p in points):
                stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
                backup = path.parent / '.backups' / (path.name + '.' + stamp + '.bak')
                backup.parent.mkdir(parents=True, exist_ok=True)
                backup.write_bytes(path.read_bytes())
                data['waypoints'] = [p for p in points if p is not None]
        validate_waypoints(data)
        if data['frame_id'] != frame:
            raise ValueError('File frame differs from recording frame')
        point = dict(point)
        number = len(data['waypoints']) + 1
        existing = {p.get('name') for p in data['waypoints']}
        while 'point_%03d' % number in existing:
            number += 1
        point['name'] = 'point_%03d' % number
        data['waypoints'].append(point)
        validate_waypoints(data)
        # YAML 每个点先显示名字，再显示坐标；保留已有其它字段及命名。
        data['waypoints'] = [dict(name=p.get('name', 'point_%03d' % i),
                                 **{k: v for k, v in p.items() if k != 'name'})
                             for i, p in enumerate(data['waypoints'], 1)]
        fd, temporary = tempfile.mkstemp(prefix=path.name + '.', dir=str(path.parent))
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as handle:
                yaml.safe_dump(data, handle, sort_keys=False, allow_unicode=True)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, str(path))
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        return point['name'], len(data['waypoints'])


def saturate_twist(v, w, max_forward, max_reverse, max_angular):
    """Scale allowed arcs together; reject a forbidden reverse command in full."""
    v, w = finite(v, 'vx'), finite(w, 'wz')
    if max_reverse == 0.0 and v < 0.0:
        # Removing only vx would turn a planned reverse arc into a different motion.
        # A requested pure spin (vx == 0) remains a valid command below.
        return 0.0, 0.0
    scale = 1.0
    if v:
        scale = min(scale, (max_forward if v > 0 else max_reverse) / abs(v))
    if w:
        scale = min(scale, max_angular / abs(w))
    return v * scale, w * scale


def compensate_turn(v, w, minimum_angular, linear_window=.03, intent_threshold=.02):
    """Add a bounded turn floor only to deliberate, near-stationary turns."""
    if abs(v) <= linear_window and intent_threshold <= abs(w) < minimum_angular:
        w = math.copysign(minimum_angular, w)
    return v, w


def motion_delta(anchor, pose):
    distance = math.hypot(pose[0]-anchor[0], pose[1]-anchor[1])
    yaw = abs(math.atan2(math.sin(pose[2]-anchor[2]), math.cos(pose[2]-anchor[2])))
    return distance, yaw


def is_stalled(anchor, pose, elapsed, stall_time, stall_distance, stall_yaw):
    distance, yaw = motion_delta(anchor, pose)
    return elapsed >= stall_time and distance < stall_distance and yaw < stall_yaw


def limit_twist(v, w, max_forward, max_reverse, max_angular, epsilon):
    """只限制线/角速度幅值；差速轮允许零半径自转，随后必须做足迹碰撞检查。"""
    v, w = finite(v, 'vx'), finite(w, 'wz')
    v = min(max_forward, max(-max_reverse, v))
    w = max(-max_angular, min(max_angular, w))
    if abs(w) <= epsilon:
        w = 0.0
    return v, w


def slew_arc(previous, target, dt, linear_limit, angular_limit):
    """Independent axis ramps: linear sign jitter must not reset angular speed."""
    if target == (0.0, 0.0):
        return target
    pv, pw = previous
    tv, tw = target
    linear_step = linear_limit * max(0.0, dt)
    angular_step = angular_limit * max(0.0, dt)
    # A requested pure spin must have exactly zero translation.
    v = 0.0 if tv == 0.0 else pv + max(-linear_step, min(linear_step, tv-pv))
    w = pw + max(-angular_step, min(angular_step, tw-pw))
    return v, w


def forward_obstacle_distance(pose, footprint, grid, velocity, threshold=100, unknown=True,
                              search_distance=0.45, front_half_angle=math.pi):
    """从足迹前/后沿到局部代价地图高代价格的距离；只检查行进方向近邻。"""
    if not velocity:
        return math.inf
    x, y, yaw = pose
    ox, oy, oa = grid['origin']
    resolution = grid['resolution']
    min_x, max_x = min(p[0] for p in footprint), max(p[0] for p in footprint)
    lateral = max(abs(p[1]) for p in footprint) + resolution
    extent = max(abs(min_x), abs(max_x)) + search_distance + resolution
    # 只遍历机器人附近的栅格，避免 10 Hz 下逐帧扫描整个局部地图。
    radius = extent + lateral
    gx, gy = math.cos(oa) * (x - ox) + math.sin(oa) * (y - oy), -math.sin(oa) * (x - ox) + math.cos(oa) * (y - oy)
    ix0 = max(0, int(math.floor((gx - radius) / resolution)))
    ix1 = min(grid['width'] - 1, int(math.ceil((gx + radius) / resolution)))
    iy0 = max(0, int(math.floor((gy - radius) / resolution)))
    iy1 = min(grid['height'] - 1, int(math.ceil((gy + radius) / resolution)))
    cy, sy = math.cos(yaw), math.sin(yaw)
    ca, sa = math.cos(oa), math.sin(oa)
    closest = math.inf
    for iy in range(iy0, iy1 + 1):
        for ix in range(ix0, ix1 + 1):
            value = grid['data'][iy * grid['width'] + ix]
            if value < threshold and not (unknown and value < 0):
                continue
            wx = ox + ca * (ix + 0.5) * resolution - sa * (iy + 0.5) * resolution - x
            wy = oy + sa * (ix + 0.5) * resolution + ca * (iy + 0.5) * resolution - y
            rx, ry = cy * wx + sy * wy, -sy * wx + cy * wy
            if velocity > 0 and abs(math.atan2(ry, rx)) > front_half_angle:
                continue
            if abs(ry) > lateral:
                continue
            distance = rx - max_x if velocity > 0 else min_x - rx
            if -resolution <= distance <= search_distance + resolution:
                closest = min(closest, max(0.0, distance - 0.5 * resolution))
    return closest


def obstacle_speed_scale(distance, stop_distance, start_distance, minimum_scale):
    if not 0 <= stop_distance < start_distance or not 0 < minimum_scale <= 1:
        raise ValueError('Invalid obstacle slowdown parameters')
    if distance <= stop_distance:
        return 0.0
    if distance >= start_distance:
        return 1.0
    ratio = (distance - stop_distance) / (start_distance - stop_distance)
    smooth = ratio * ratio * (3 - 2 * ratio)
    return minimum_scale + (1 - minimum_scale) * smooth


def parse_footprint(value):
    # costmap 的 dynamic_reconfigure 会把启动时的数组参数回写成字符串。
    if isinstance(value, str):
        value = yaml.safe_load(value)
    if not isinstance(value, list) or len(value) < 3:
        raise ValueError('footprint requires >=3 convex vertices')
    result = [(finite(p[0], 'footprint x'), finite(p[1], 'footprint y')) for p in value if len(p) == 2]
    if len(result) != len(value):
        raise ValueError('Invalid footprint vertex')
    turns = []
    for i, p in enumerate(result):
        q, r = result[(i+1)%len(result)], result[(i+2)%len(result)]
        turns.append((q[0]-p[0])*(r[1]-q[1])-(q[1]-p[1])*(r[0]-q[0]))
    if not (all(t > 0 for t in turns) or all(t < 0 for t in turns)):
        raise ValueError('Guard requires a nondegenerate convex footprint')
    return result


def intersect(a, b):
    """凸多边形 SAT，相接也算碰撞；可检查细小障碍格与旋转矩形的边缘。"""
    for polygon in (a, b):
        for i, p in enumerate(polygon):
            q = polygon[(i + 1) % len(polygon)]
            axis = (-(q[1] - p[1]), q[0] - p[0])
            pa = [x * axis[0] + y * axis[1] for x, y in a]
            pb = [x * axis[0] + y * axis[1] for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


def footprint_clear(pose, footprint, grid, threshold=100, unknown=True):
    """网格字典的 origin=(x,y,yaw)，data 为行优先 OccupancyGrid。"""
    x, y, yaw = pose
    ox, oy, oa = grid['origin']
    ca, sa = math.cos(oa), math.sin(oa)
    c, s = math.cos(yaw), math.sin(yaw)
    polygon = []
    for px, py in footprint:
        wx, wy = x + c * px - s * py - ox, y + s * px + c * py - oy
        polygon.append(((ca * wx + sa * wy) / grid['resolution'],
                        (-sa * wx + ca * wy) / grid['resolution']))
    x0, x1 = math.floor(min(p[0] for p in polygon)), math.floor(max(p[0] for p in polygon))
    y0, y1 = math.floor(min(p[1] for p in polygon)), math.floor(max(p[1] for p in polygon))
    if x0 < 0 or y0 < 0 or x1 >= grid['width'] or y1 >= grid['height']:
        return False
    for iy in range(y0, y1 + 1):
        for ix in range(x0, x1 + 1):
            value = grid['data'][iy * grid['width'] + ix]
            if value >= threshold or (value < 0 and unknown):
                cell = [(ix, iy), (ix + 1, iy), (ix + 1, iy + 1), (ix, iy + 1)]
                if intersect(polygon, cell):
                    return False
    return True


def swept_clear(pose, v, w, footprint, grid, horizon, threshold=100, unknown=True):
    extent = max(math.hypot(*p) for p in footprint)
    # 同时限制平移和足迹旋转边缘的离散间隔，避免跨过单格障碍物。
    speed = abs(v) + abs(w) * extent
    dt = min(0.1, grid['resolution'] / (2 * speed)) if speed else 0.1
    steps = max(1, math.ceil(horizon / dt))
    x, y, yaw = pose
    for step in range(steps + 1):
        t = horizon * step / steps
        if abs(w) < 1e-9:
            p = (x + v * t * math.cos(yaw), y + v * t * math.sin(yaw), yaw)
        else:
            p = (x + v / w * (math.sin(yaw + w * t) - math.sin(yaw)),
                 y - v / w * (math.cos(yaw + w * t) - math.cos(yaw)), yaw + w * t)
        if not footprint_clear(p, footprint, grid, threshold, unknown):
            return False
    return True


def project_safe_twist(pose, v, w, footprint, grid, horizon, threshold=100,
                       unknown=True, iterations=6):
    """Scale both Twist components along the same arc to the safest usable prefix.

    The path for a scaled constant twist is a prefix of the original arc over
    the same horizon. Never project through an occupied starting footprint.
    """
    if not footprint_clear(pose, footprint, grid, threshold, unknown):
        return 0.0, 0.0, 'current footprint occupied/unknown/outside costmap'
    if swept_clear(pose, v, w, footprint, grid, horizon, threshold, unknown):
        return v, w, ''
    low, high = 0.0, 1.0
    for _ in range(iterations):
        fraction = (low + high) / 2.0
        if swept_clear(pose, v*fraction, w*fraction, footprint, grid,
                       horizon, threshold, unknown):
            low = fraction
        else:
            high = fraction
    if low <= 0.0:
        return 0.0, 0.0, 'no safe motion in predicted footprint'
    return v*low, w*low, 'scaled to %.0f%% for predicted footprint' % (100*low)
