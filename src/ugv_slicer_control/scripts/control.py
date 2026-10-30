#!/usr/bin/env python3
"""Explicit, short-lived control handoff through the existing Slicer protocol."""
import argparse
import json
import math
import socket
import sys
import time
from slicer_manual import INTENT_TOPIC, STATE_TOPIC, parse_manual_state, release_intent, takeover_intent


def read_state(message, owner):
    header = getattr(message, '_connection_header', {}) or {}
    if header.get('callerid') != owner or header.get('latching', '0') != '0':
        raise ValueError('manual state must come from the nonlatched mux owner')
    return parse_manual_state(message.data)


def fresh_owned(sample, now, maximum_age, publishers, owner):
    return (sample is not None and 0 <= now - sample[0] <= maximum_age
            and publishers == [owner])


def release_allowed(statuses):
    return statuses is not None and all(item.status not in (0, 1, 6, 7) for item in statuses)


def acknowledged(state, request, before, held):
    generation = before['takeover_generation'] + (1 if held else 0)
    return (state['mux_instance_id'] == before['mux_instance_id']
            and state['takeover_generation'] == generation
            and state['last_intent_id'] == request['intent_id']
            and state['last_intent_action'] == request['action']
            and state['last_intent_accepted'] and state['manual_hold'] is held)


def main():
    import rosgraph
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('state', 'release', 'hold'))
    parser.add_argument('--namespace', default='/ugv')
    for option in ('mux-owner', 'intent-topic', 'state-topic', 'planner'):
        parser.add_argument('--' + option)
    parser.add_argument('--timeout', type=float)
    parser.add_argument('--freshness', type=float)
    args = parser.parse_args(rosgraph.myargv()[1:])
    namespace = args.namespace.rstrip('/')
    if not namespace.startswith('/') or '..' in namespace.split('/') or not namespace:
        raise ValueError('an absolute node namespace is required')
    socket.setdefaulttimeout(args.timeout or 3.)
    rosgraph.Master(namespace + '/control').getPid()  # Fail promptly without a master.
    argv = [arg for arg in sys.argv if not arg.startswith(('__name:=', '__ns:='))]
    # rospy caches its initial caller namespace on import, before init_node(argv).
    # Set the real process arguments first, including in a normal root-namespace shell.
    sys.argv[:] = argv + ['__ns:=' + namespace, '__name:=control']
    import rospy
    from actionlib_msgs.msg import GoalID, GoalStatusArray
    from std_msgs.msg import String
    rospy.init_node('control', argv=sys.argv, anonymous=False)
    if rospy.get_name() != namespace + '/control':
        raise RuntimeError('control node identity does not match the requested namespace')
    options = {}
    defaults = dict(mux_owner='/slicer_velocity_mux', intent_topic=INTENT_TOPIC,
                    state_topic=STATE_TOPIC, planner=namespace + '/move_base', timeout=3., freshness=.5)
    for key, default in defaults.items():
        value = getattr(args, key)
        options[key] = rospy.get_param('~' + key, default) if value is None else value
    timeout, freshness = float(options['timeout']), float(options['freshness'])
    if not all(math.isfinite(value) and value > 0 for value in (timeout, freshness)):
        raise ValueError('timeout and freshness must be finite and positive')
    socket.setdefaulttimeout(timeout)
    owner, planner = options['mux_owner'], options['planner'].rstrip('/')
    state_topic, intent_topic = [rospy.resolve_name(options[key]) for key in ('state_topic', 'intent_topic')]
    status_topic, cancel_topic = planner + '/status', planner + '/cancel'
    master, samples = rosgraph.Master(rospy.get_name()), {}

    def receive_state(message):
        try:
            samples['state'] = time.monotonic(), read_state(message, owner)
        except (ValueError, TypeError):
            pass

    def receive_status(message):
        header = getattr(message, '_connection_header', {}) or {}
        # Action status may be latched; its ROS timestamp must still be current.
        age = (rospy.Time.now() - message.header.stamp).to_sec()
        if header.get('callerid') == planner and 0 <= age <= freshness:
            samples['status'] = time.monotonic(), message.status_list, message.header.stamp

    def publishers(topic):
        return dict(master.getSystemState()[0]).get(topic, [])

    def ready(key, topic, expected):
        registered = publishers(topic)
        sample = samples.get(key)
        if not fresh_owned(sample, time.monotonic(), freshness, registered, expected):
            return False
        return key != 'status' or 0 <= (rospy.Time.now() - sample[2]).to_sec() <= freshness

    def wait_for(predicate, explanation):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.02)
        raise RuntimeError(explanation)

    subscribers = [rospy.Subscriber(state_topic, String, receive_state, queue_size=10),
                   rospy.Subscriber(status_topic, GoalStatusArray, receive_status, queue_size=10)]
    wait_for(lambda: ready('state', state_topic, owner), 'no fresh state from the unique mux owner')
    before = samples['state'][1]
    if args.command == 'state':
        print(json.dumps(before, sort_keys=True))
        return 0
    held = args.command == 'hold'
    if not held:
        wait_for(lambda: ready('status', status_topic, planner), 'no fresh owned move_base status')
        if not release_allowed(samples['status'][1]):
            raise RuntimeError('release refused: move_base has a pending or active goal; use hold first')
        if before['teleop_fresh'] or before['emergency_stop_latched']:
            raise RuntimeError('release refused: teleop activity or emergency stop remains')
    publisher = rospy.Publisher(intent_topic, String, queue_size=1, latch=False)
    wait_for(lambda: publisher.get_num_connections() > 0, 'manual intent subscriber unavailable')
    # Observers (rosbag/rostopic) may subscribe too. Only the mux can return the
    # fresh, owned, instance/generation-bound ACK checked below.
    if owner not in dict(master.getSystemState()[1]).get(intent_topic, []):
        raise RuntimeError('manual intent must have the expected mux subscriber')
    if not held:
        if not ready('status', status_topic, planner) or not release_allowed(samples['status'][1]):
            raise RuntimeError('release refused: move_base status changed or became stale')
    if not ready('state', state_topic, owner):
        raise RuntimeError('manual state became stale or its publisher changed')
    current = samples['state'][1]
    if held:
        before = current
    elif any(current[key] != before[key] for key in ('mux_instance_id', 'takeover_generation')):
        raise RuntimeError('release refused: mux instance or takeover generation changed; explicitly run release again')
    request = takeover_intent() if held else release_intent(before)
    publisher.publish(String(data=json.dumps(request, sort_keys=True)))

    def matched():
        if not ready('state', state_topic, owner):
            return False
        state = samples['state'][1]
        if state['last_intent_id'] == request['intent_id'] and not state['last_intent_accepted']:
            raise RuntimeError(state['last_intent_reason'])
        return acknowledged(state, request, before, held)

    wait_for(matched, 'manual handoff ACK or postcondition timed out; request was not retried')
    print(json.dumps(samples['state'][1], sort_keys=True))
    if held:
        try:
            cancel = rospy.Publisher(cancel_topic, GoalID, queue_size=1, latch=False)
            wait_for(lambda: cancel.get_num_connections() > 0, 'move_base cancel subscriber unavailable')
            cancel_at = time.monotonic()
            cancel.publish(GoalID())  # All goals on this action only, after confirmed manual hold.
            wait_for(lambda: ready('status', status_topic, planner)
                     and samples['status'][0] >= cancel_at and release_allowed(samples['status'][1]),
                     'move_base cancellation status ACK timed out')
        except Exception as error:
            raise RuntimeError('held confirmed; cancel unconfirmed: ' + str(error)) from error
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as error:
        print('control: ' + str(error), file=sys.stderr)
        sys.exit(1)
