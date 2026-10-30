#!/usr/bin/env python3
"""Finite 0x16 field comparison through Slicer Guard's /cmd_vel/nav_raw only.

For operator-confirmed field use. The operator performs release/hold externally.
Cancel the active navigation goal first; keep move_base running for Guard's costmap.
steady: +0.08 rad/s for 12s; steady-negative: -0.08 rad/s for 12s;
alternate: six 2s legs, +0.08/-0.08 rad/s.
Both have 2s zero before and after, at 20Hz, with all linear velocities zero.
Requires fresh driver ready true; stops on false or stopped ready updates.
"""
import argparse
import math
import sys
import time


TOPIC = '/cmd_vel/nav_raw'


def command_at(case, elapsed):
    """Return angular.z at a monotonic elapsed time; all other axes stay zero."""
    if not 2. <= elapsed < 14.:
        return 0.
    if case == 'steady':
        return .08
    if case == 'steady-negative':
        return -.08
    return .08 if int((elapsed - 2.) / 2.) % 2 == 0 else -.08


def run_schedule(case, publish, monotonic=time.monotonic, sleep=time.sleep,
                 should_stop=lambda: False):
    """Keep one publisher for the finite sequence, and attempt zero on any exit."""
    start, tick = monotonic(), 0
    try:
        while not should_stop():
            elapsed = monotonic() - start
            if elapsed >= 16.:
                break
            publish(command_at(case, elapsed))
            tick = max(tick + 1, int((monotonic() - start) * 20.) + 1)
            sleep(max(0., start + tick / 20. - monotonic()))
    finally:
        publish(0.)


def ready_fresh(value, received, now, timeout):
    return value is True and 0. <= now - received <= timeout


def ready_update(previous, value, received, running):
    """A false observation during this run cannot be erased by a later true."""
    return value, received, previous[2] or (running and not value)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', choices=('steady', 'steady-negative', 'alternate'), nargs='?', default='steady')
    args = parser.parse_args(argv)
    print('FIELD COMMAND: topic={} case={} 20Hz; linear.xyz=0, angular.xy=0'.format(TOPIC, args.case),
          flush=True)
    print('0-2s: wz=0; 2-14s: {}; 14-16s: wz=0; final zero on exit'.format(
        'wz=+0.08 rad/s' if args.case == 'steady' else
        'wz=-0.08 rad/s' if args.case == 'steady-negative' else
        'wz=+0.08,-0.08,+0.08,-0.08,+0.08,-0.08 rad/s, 2s each'), flush=True)
    print('Run only after field confirmation and goal cancellation; keep move_base costmap running. '
          'Operator performs release/hold externally.', flush=True)

    # Lazy imports keep --help and pure timing tests independent of ROS.
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool

    rospy.init_node('ugv_probe_body_turn', anonymous=True, disable_signals=True)
    publisher = ready_subscriber = raw_subscriber = None
    try:
        if rospy.resolve_name(TOPIC) != TOPIC:
            raise RuntimeError('raw topic remapping is not allowed')
        # The driver publishes ready every control loop. Reuse its existing
        # telemetry timeout rather than introduce a new field fault threshold.
        ready_timeout = float(rospy.get_param(
            '/slicer_c30d_driver/provenance/effective_config/telemetry_timeout_s'))
        if not math.isfinite(ready_timeout) or ready_timeout <= 0.:
            raise RuntimeError('driver telemetry timeout is unavailable or invalid')
        ready, running = (False, -math.inf, False), False

        def observe_ready(msg):
            nonlocal ready
            ready = ready_update(ready, msg.data, time.monotonic(), running)

        conflicting_source = None

        def observe_raw(msg):
            nonlocal conflicting_source
            source = (msg._connection_header or {}).get('callerid')
            if source != rospy.get_name():
                conflicting_source = source or 'unknown'

        ready_subscriber = rospy.Subscriber('/slicer_c30d_driver/ready', Bool, observe_ready, queue_size=1)
        raw_subscriber = rospy.Subscriber(TOPIC, Twist, observe_raw, queue_size=1)
        publisher = rospy.Publisher(TOPIC, Twist, queue_size=1, latch=False)
        deadline = time.monotonic() + 5.
        # Our own raw observer counts as one connection; require another sink.
        while (publisher.get_num_connections() < 2 or
               not ready_fresh(*ready[:2], time.monotonic(), ready_timeout)):
            if conflicting_source is not None:
                raise RuntimeError('another raw source sent a command: ' + conflicting_source)
            if rospy.is_shutdown() or time.monotonic() >= deadline:
                raise RuntimeError('raw subscriber and fresh driver ready true required within 5 seconds')
            time.sleep(.05)

        def publish(wz):
            msg = Twist()
            msg.angular.z = wz
            publisher.publish(msg)

        def should_stop():
            if rospy.is_shutdown():
                return True
            if conflicting_source is not None:
                raise RuntimeError('another raw source sent a command: ' + conflicting_source)
            if ready[2] or not ready_fresh(*ready[:2], time.monotonic(), ready_timeout):
                raise RuntimeError('driver ready false or stale; ending probe')
            return False

        print('START: raw subscriber connected, fresh driver ready true; finite 16s sequence begins.', flush=True)
        running = True
        run_schedule(args.case, publish, should_stop=should_stop)
        return 0
    except KeyboardInterrupt:
        print('Interrupted; sending final zero.', file=sys.stderr, flush=True)
        return 130
    except Exception as exc:
        print('Probe stopped: {}'.format(exc), file=sys.stderr, flush=True)
        return 1
    finally:
        if publisher is not None:
            try:
                publisher.publish(Twist())
                time.sleep(.1)  # Allow the bounded final zero to leave the queue.
            finally:
                publisher.unregister()
        if ready_subscriber is not None:
            ready_subscriber.unregister()
        if raw_subscriber is not None:
            raw_subscriber.unregister()
        rospy.signal_shutdown('finite body turn probe closed')


if __name__ == '__main__':
    raise SystemExit(main())
