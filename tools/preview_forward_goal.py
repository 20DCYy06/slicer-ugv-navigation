#!/usr/bin/env python3
"""Freeze one forward goal in odom and call GetPlan only; never send an Action."""
import argparse
import ast
from collections import namedtuple
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys


FrozenGoal = namedtuple('FrozenGoal', 'x y yaw')
ACTION = '/ugv/move_base'
COSTMAP_TOPIC = ACTION + '/global_costmap/costmap'
PLAN_SERVICE = ACTION + '/make_plan'
FOOTPRINT_PARAM = ACTION + '/global_costmap/footprint'


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise argparse.ArgumentTypeError('must be finite')
    return value


def positive(value):
    value = finite(value)
    if value <= 0.0:
        raise argparse.ArgumentTypeError('must be greater than zero')
    return value


def nonnegative(value):
    value = finite(value)
    if value < 0.0:
        raise argparse.ArgumentTypeError('must be zero or greater')
    return value


def arguments(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--distance', type=positive, default=10.0, help='forward distance in metres (default: 10)')
    parser.add_argument('--output', required=True, type=Path, help='new JSON file for this preview; never overwrite')
    parser.add_argument('--margin', type=nonnegative, default=None,
                        help='window edge margin in metres; default: live footprint radius + padding + 0.05')
    return parser.parse_args(argv)


def freeze_forward_goal(x, y, yaw, distance):
    """Return immutable absolute coordinates from one planar odom sample."""
    return FrozenGoal(x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw)


def footprint_radius(vertices):
    if len(vertices) < 3:
        raise ValueError('global_costmap footprint needs at least three vertices')
    radii = []
    for vertex in vertices:
        if len(vertex) != 2 or not all(math.isfinite(float(value)) for value in vertex):
            raise ValueError('invalid global_costmap footprint vertex')
        radii.append(math.hypot(float(vertex[0]), float(vertex[1])))
    return max(radii)


def check_window(goal, origin_x, origin_y, width, height, resolution, margin):
    """Check the actual axis-aligned OccupancyGrid extent, including edge margin."""
    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise ValueError('invalid global costmap dimensions or resolution')
    max_x = origin_x + width * resolution
    max_y = origin_y + height * resolution
    clearances = {'left': goal.x - origin_x, 'right': max_x - goal.x,
                  'bottom': goal.y - origin_y, 'top': max_y - goal.y}
    inside = origin_x <= goal.x < max_x and origin_y <= goal.y < max_y
    enough_margin = min(clearances.values()) + 1e-9 >= margin
    reason = ('target outside actual global costmap window' if not inside else
              'target has less than {:.6f}m window edge margin'.format(margin) if not enough_margin else
              'target is inside actual global costmap window with the required margin')
    return {'ok': inside and enough_margin, 'reason': reason,
            'bounds_m': {'min_x': origin_x, 'max_x': max_x, 'min_y': origin_y, 'max_y': max_y},
            'clearances_m': clearances, 'required_margin_m': margin}


def quaternion_values(q):
    return [q.x, q.y, q.z, q.w]


def normalized_quaternion(q):
    values = quaternion_values(q)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('quaternion contains a nonfinite value')
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-12:
        raise ValueError('quaternion has zero length')
    return [value / norm for value in values]


def quaternion_yaw(q):
    x, y, z, w = normalized_quaternion(q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_record(pose):
    p = pose.pose.position
    return {'frame_id': pose.header.frame_id, 'stamp': pose.header.stamp.to_sec(),
            'position': {'x': p.x, 'y': p.y, 'z': p.z},
            'orientation_xyzw': quaternion_values(pose.pose.orientation)}


def preview(args, record):
    # Deliberately lazy: --help and the pure geometry tests do not require ROS.
    import rospy
    import tf2_ros
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid
    from nav_msgs.srv import GetPlan, GetPlanRequest

    rospy.init_node('ugv_preview_forward_goal', anonymous=True, disable_signals=True)
    try:
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        transform = buffer.lookup_transform('odom', 'base_link', rospy.Time(0), rospy.Duration(3.0))
        age = (rospy.Time.now() - transform.header.stamp).to_sec()
        if transform.header.stamp.is_zero() or abs(age) > 0.5:
            raise ValueError('odom <- base_link TF is not fresh: age={:.3f}s (limit 0.5s)'.format(age))
        position = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = quaternion_yaw(rotation)
        goal = freeze_forward_goal(position.x, position.y, yaw, args.distance)
        record['tf_sample'] = {'stamp': transform.header.stamp.to_sec(), 'age_s': age,
                               'translation': {'x': position.x, 'y': position.y, 'z': position.z},
                               'orientation_xyzw': quaternion_values(rotation)}
        record['start'] = {'frame_id': 'odom', 'x': position.x, 'y': position.y, 'yaw': yaw}
        record['frozen_goal'] = dict(goal._asdict(), frame_id='odom')
        print('Frozen odom goal: x={:.9f} y={:.9f} yaw={:.9f} rad'.format(*goal), flush=True)

        vertices = rospy.get_param(FOOTPRINT_PARAM)
        if isinstance(vertices, str):
            vertices = ast.literal_eval(vertices)
        padding = float(rospy.get_param(ACTION + '/global_costmap/footprint_padding', 0.0))
        if not math.isfinite(padding) or padding < 0.0:
            raise ValueError('global_costmap footprint padding must be finite and nonnegative')
        radius = footprint_radius(vertices)
        default_margin = radius + padding + 0.05
        margin = default_margin if args.margin is None else args.margin
        record['footprint'] = {'parameter': FOOTPRINT_PARAM, 'vertices': vertices, 'padding_m': padding,
                               'circumscribed_radius_m': radius, 'default_margin_m': default_margin}

        grid = rospy.wait_for_message(COSTMAP_TOPIC, OccupancyGrid, timeout=5.0)
        info = grid.info
        origin = info.origin
        record['costmap'] = {'topic': COSTMAP_TOPIC, 'frame_id': grid.header.frame_id,
                             'stamp': grid.header.stamp.to_sec(), 'resolution_m': info.resolution,
                             'width_cells': info.width, 'height_cells': info.height,
                             'origin': {'x': origin.position.x, 'y': origin.position.y, 'z': origin.position.z,
                                        'orientation_xyzw': quaternion_values(origin.orientation)}}
        if grid.header.frame_id != 'odom':
            raise ValueError('global costmap frame must be odom; got {!r}'.format(grid.header.frame_id))
        qx, qy, qz, qw = normalized_quaternion(origin.orientation)
        if max(abs(qx), abs(qy), abs(qz)) > 1e-8:
            raise ValueError('rotated global costmap origin is unsupported; only identity rotation is supported')
        boundary = check_window(goal, origin.position.x, origin.position.y,
                                info.width, info.height, info.resolution, margin)
        record['window_check'] = boundary
        print('Window margin {:.6f}m (footprint radius {:.6f}m + padding {:.6f}m + 0.05m default)'.format(
            margin, radius, padding), flush=True)
        if not boundary['ok']:
            record['status'] = 'window_rejected'
            print(boundary['reason'], file=sys.stderr)
            return 1

        start = PoseStamped()
        start.header.frame_id = 'odom'
        start.header.stamp = transform.header.stamp
        start.pose.position.x = position.x
        start.pose.position.y = position.y
        start.pose.position.z = position.z
        start.pose.orientation = rotation
        target = PoseStamped()
        target.header.frame_id = 'odom'
        target.header.stamp = transform.header.stamp
        target.pose.position.x = goal.x
        target.pose.position.y = goal.y
        target.pose.orientation.z = math.sin(goal.yaw / 2.0)
        target.pose.orientation.w = math.cos(goal.yaw / 2.0)
        rospy.wait_for_service(PLAN_SERVICE, timeout=5.0)
        record['plan']['requested'] = True
        response = rospy.ServiceProxy(PLAN_SERVICE, GetPlan)(GetPlanRequest(start=start, goal=target, tolerance=0.0))
        record['plan'].update(frame_id=response.plan.header.frame_id, stamp=response.plan.header.stamp.to_sec(),
                              pose_count=len(response.plan.poses), poses=[pose_record(p) for p in response.plan.poses])
        if not response.plan.poses:
            record['status'] = 'empty_plan'
            print('GetPlan returned no poses; no send command is offered.', file=sys.stderr)
            return 1
        command = ('rosrun ugv_navigation goal.py send --frame odom '
                   '--x {:.9f} --y {:.9f} --yaw {:.9f} --timeout 180').format(*goal)
        record['send_command'] = command
        record['status'] = 'plan_available'
        print('GetPlan: {} poses. No Action goal sent.'.format(len(response.plan.poses)), flush=True)
        print('After external checks and an on-site operator release, copy this fixed-coordinate command:', flush=True)
        print(command, flush=True)
        return 0
    finally:
        rospy.signal_shutdown('preview finished')


def main(argv=None):
    args = arguments(sys.argv[1:] if argv is None else argv)
    record = {'schema_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
              'preview_only': True, 'requested_distance_m': args.distance, 'status': 'error',
              'plan': {'service': PLAN_SERVICE, 'requested': False, 'pose_count': 0, 'poses': []}}
    try:
        with args.output.open('x', encoding='utf-8') as output:
            try:
                result = preview(args, record)
            except KeyboardInterrupt:
                record['error'] = 'interrupted'
                result = 130
            except Exception as error:
                record['error'] = '{}: {}'.format(type(error).__name__, error)
                print(record['error'], file=sys.stderr)
                result = 1
            json.dump(record, output, ensure_ascii=False, allow_nan=False, indent=2)
            output.write('\n')
        print('Saved preview: {}'.format(args.output), flush=True)
        return result
    except OSError as error:
        print('Cannot write preview JSON: {}'.format(error), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
