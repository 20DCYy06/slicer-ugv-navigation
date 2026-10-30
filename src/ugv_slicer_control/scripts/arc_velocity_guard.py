#!/usr/bin/env python3
"""Navigation velocity smoothing with forward-only, swept-checked turn compensation."""
import math
from pathlib import Path
import sys
import threading
import time
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from navigation_core import saturate_twist, slew_arc, compensate_turn, swept_clear, parse_footprint, planner_limits


def main():
    rospy.init_node('slicer_nav_arc_guard')
    p = rospy.get_param('~')
    lio_timeout = float(p.get('lio_timeout', .5))
    if not math.isfinite(lio_timeout) or lio_timeout <= 0:
        raise ValueError('lio_timeout must be finite and positive')
    allow_patrol_spin = p.get('allow_patrol_spin', True)
    if type(allow_patrol_spin) is not bool:
        raise ValueError('allow_patrol_spin must be a boolean')
    with open(p['config_file'], encoding='utf-8') as stream:
        limits = yaml.safe_load(stream)['navigation']
    planner_ns = str(p.get('planner_namespace', '/move_base')).rstrip('/')
    if not planner_ns.startswith('/') or '..' in planner_ns.split('/'):
        raise ValueError('explicit absolute planner namespace required')
    plugin = rospy.get_param(planner_ns + '/base_local_planner')
    if not isinstance(plugin, str):
        raise ValueError('base_local_planner must name a supported plugin')
    planner = rospy.get_param(planner_ns + '/' + plugin.rsplit('/', 1)[-1], {})
    # Source physics and the selected planner both constrain this outlet. A
    # commissioning planner may deliberately request lower limits than daily nav.
    limits = planner_limits(limits, plugin, planner)
    rospy.loginfo('Guard planner %s limits: forward=%.3f reverse=%.3f angular=%.3f',
                  plugin, limits['max_forward'], limits['max_reverse'], limits['max_angular'])
    with open(p['control_config_file'], encoding='utf-8') as stream:
        control = yaml.safe_load(stream)
    base = rospy.get_param(planner_ns + '/local_costmap/robot_base_frame', control['base_frame'])
    compensation_enabled = p.get('turn_compensation_enabled', True)
    if type(compensation_enabled) is not bool:
        raise ValueError('turn_compensation_enabled must be a boolean')
    start_pwm, min_angular = 0., 0.
    if compensation_enabled:
        start_pwm = float(limits['turn_start_pwm_permille'])
        gain = float(control['turn_gain_permille_per_rad_s'])
        if not math.isfinite(gain) or gain <= 0:
            raise ValueError('Invalid turn PWM gain')
        min_angular = start_pwm / gain
        if not (0 < start_pwm <= control['ros_driver_pwm_limit_permille'] and
                min_angular <= limits['max_angular']):
            raise ValueError('Turn-start PWM exceeds effective planner angular or driver cap')
        rospy.loginfo('Turn compensation: %.0f permille = %.3frad/s floor; reverse limit=%.3fm/s',
                      start_pwm, min_angular, limits['max_reverse'])
    lock, sources = threading.Lock(), {}
    latest_grid, footprint = [None, -math.inf], [None]
    latest_lio = [-math.inf, None]  # monotonic receive time and advancing source stamp
    listener = tf.TransformListener()
    pub = rospy.Publisher(p['output_topic'], Twist, queue_size=1)
    reason_pub = rospy.Publisher('~hold_reason', String, queue_size=1, latch=True)
    adjustment_pub = rospy.Publisher('~adjustment', String, queue_size=1, latch=True)

    def receive(msg, source):
        value = (msg.linear.x, msg.angular.z)
        valid = all(math.isfinite(x) for x in value)
        with lock:
            sources[source] = (time.monotonic(), value if valid else (0., 0.), valid)

    def lio(msg):
        received, stamp = time.monotonic(), msg.header.stamp
        age = (rospy.Time.now()-stamp).to_sec()
        if stamp.to_sec() <= 0 or not math.isfinite(age) or age < 0 or age > lio_timeout:
            return
        with lock:
            # Repeated old headers must not keep a stopped LIO source alive.
            if latest_lio[1] is None or stamp > latest_lio[1]:
                latest_lio[:] = [received, stamp]

    def costmap(msg):
        if msg.info.resolution <= 0 or len(msg.data) != msg.info.width*msg.info.height:
            return
        q = msg.info.origin.orientation
        grid = dict(width=msg.info.width, height=msg.info.height, data=msg.data,
                    resolution=msg.info.resolution, frame=msg.header.frame_id,
                    origin=(msg.info.origin.position.x, msg.info.origin.position.y,
                            tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w))[2]))
        with lock:
            latest_grid[:] = [grid, time.monotonic()]

    def motion_error(grid, grid_at, now, target, output):
        """Every nonzero output requires fresh context and a clear swept footprint."""
        try:
            with lock:
                lio_received, lio_stamp = latest_lio
            if now-lio_received > lio_timeout:
                return 'motion needs fresh LIO odometry'
            lio_age = (rospy.Time.now()-lio_stamp).to_sec()
            if lio_age < 0 or lio_age > lio_timeout:
                return 'motion needs a current advancing LIO stamp'
            if grid is None or now-grid_at > p['costmap_timeout']:
                return 'motion needs a fresh local grid'
            stamp = listener.getLatestCommonTime(grid['frame'], base)
            if (rospy.Time.now()-stamp).to_sec() > p['tf_timeout']:
                return 'motion needs fresh base TF'
            position, quat = listener.lookupTransform(grid['frame'], base, stamp)
            pose = (position[0], position[1], tf.transformations.euler_from_quaternion(quat)[2])
            if footprint[0] is None:
                footprint[0] = parse_footprint(rospy.get_param(planner_ns + '/local_costmap/footprint'))
            horizon = max(p['compensation_horizon'],
                          abs(target[0])/limits['accel_linear']+p['command_timeout'],
                          abs(target[1])/limits['accel_angular']+p['command_timeout'])
            if not all(swept_clear(pose, *command, footprint[0], grid, horizon) for command in (target, output)):
                return 'swept footprint occupied/unknown/outside grid'
        except (tf.Exception, ValueError, KeyError) as exc:
            return str(exc)
        return ''

    rospy.Subscriber(p.get('lio_topic', '/lio/odom'), Odometry, lio, queue_size=1)
    rospy.Subscriber(p['input_topic'], Twist, receive, callback_args='nav', queue_size=1)
    if allow_patrol_spin:
        rospy.Subscriber(p.get('spin_input_topic', '/cmd_vel/patrol_spin_raw'), Twist,
                         receive, callback_args='spin', queue_size=1)
    rospy.Subscriber(p['costmap_topic'], OccupancyGrid, costmap, queue_size=1)
    previous, previous_at, last_reason, last_note = (0., 0.), time.monotonic(), None, None
    try:
        while not rospy.is_shutdown():
            now = time.monotonic()
            with lock:
                nav, spin = sources.get('nav'), sources.get('spin')
                grid, grid_at = latest_grid
            sample = (spin if spin and now-spin[0] <= p['command_timeout']
                      and spin[1] != (0., 0.) else nav)
            reason, note, bounded, target = '', '', (0., 0.), (0., 0.)
            if sample is None or now-sample[0] > p['command_timeout']:
                reason = 'command missing/stale'
            elif not sample[2]:
                reason = 'non-finite input'
            elif limits['max_reverse'] == 0 and sample[1][0] < 0:
                # A reverse arc is not a pure-spin request. Reject it before any
                # turn floor or collision fallback can preserve its angular part.
                reason = 'negative body vx disallowed; full command stopped'
            else:
                bounded = saturate_twist(*sample[1], limits['max_forward'],
                                         limits['max_reverse'], limits['max_angular'])
                target = (compensate_turn(*bounded, min_angular, p['turning_linear_window'],
                                          p['angular_intent_threshold'])
                          if compensation_enabled else bounded)
                if target != bounded:
                    note = 'turn-start floor %.0f permille (%.3frad/s)' % (start_pwm, min_angular)
            # Explicit stop and expired commands stop immediately. Nonzero commands
            # retain their curvature under saturation, including zero-radius turns.
            output = slew_arc(previous, target, min(.1, now-previous_at),
                              limits['accel_linear'], limits['accel_angular'])
            if target != (0., 0.) or output != (0., 0.):
                error = motion_error(grid, grid_at, now, target, output)
                if error:
                    fallback = slew_arc(previous, bounded, min(.1, now-previous_at),
                                        limits['accel_linear'], limits['accel_angular'])
                    if target != bounded and not motion_error(grid, grid_at, now, bounded, fallback):
                        output = fallback
                        note = 'turn boost blocked by footprint; unboosted angular command retained'
                    else:
                        output, reason = (0., 0.), error
            msg = Twist()
            msg.linear.x, msg.angular.z = output
            pub.publish(msg)
            if reason != last_reason:
                reason_pub.publish(String(data=reason))
                last_reason = reason
            if note != last_note:
                adjustment_pub.publish(String(data=note))
                last_note = note
            if reason:
                rospy.logwarn_throttle(3., 'Navigation compensation held stopped: %s', reason)
            elif note:
                rospy.loginfo_throttle(3., 'Navigation compensation: %s; input=%s output=%s', note, sample[1], output)
            previous, previous_at = output, now
            time.sleep(1. / p['publish_hz'])
    finally:
        pub.publish(Twist())


if __name__ == '__main__':
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
