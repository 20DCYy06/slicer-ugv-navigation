#!/usr/bin/env python3
"""Isolated Docker verification; synthetic state only on a private ROS master.

Run using communication/run.sh check (--network none, no hardware devices).
Never run the synthetic publisher on a production robot ROS master.
"""
import os
from pathlib import Path
import signal
import statistics
import subprocess
import tempfile
import time
import xmlrpc.client
import yaml


def until(predicate, seconds, description):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.02)
    raise AssertionError('Timeout: '+description)


def main():
    if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11311':
        raise RuntimeError('Verification requires the isolated loopback ROS master')
    here = Path(__file__).resolve().parent
    routes = yaml.safe_load((here/'config/relay.yaml').read_text())['routes']
    sync = yaml.safe_load((here/'config/sync_topics.yaml').read_text())['sync_topics']
    assert {r['target'] for r in routes} == {
        '/ugv/odom', '/ugv/pose', '/ugv/chassis_ready', '/ugv/mission/state'}
    assert {r['target'] for r in routes} <= set(sync) and '/uav/test' in sync
    assert not any('cmd_vel' in name or name in ('/tf', '/tf_static') for name in sync)
    processes = []
    try:
        with tempfile.TemporaryDirectory(prefix='ugv-relay-check-') as directory:
            with open(Path(directory)/'master.log', 'w') as log:
                processes.append(subprocess.Popen(['roscore'], stdout=log, stderr=subprocess.STDOUT))
                proxy = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
                def ready():
                    try:
                        return proxy.getPid('/ugv_relay_check')[0] == 1
                    except OSError:
                        return False
                until(ready, 10, 'private ROS master')
                import rospy
                from roslib.message import get_message_class
                rospy.init_node('ugv_relay_check', disable_signals=True)
                inputs, outputs, received, history = {}, {}, {}, {}
                subscribers, publishers = [], []
                for route in routes:
                    source, target = route['source'], route['target']
                    def capture(msg, key):
                        received[key] = (bytes(msg._buff), time.monotonic())
                        history.setdefault(key, []).append(received[key])
                    subscribers.append(rospy.Subscriber(source, rospy.AnyMsg, capture, callback_args=source))
                    subscribers.append(rospy.Subscriber(target, rospy.AnyMsg, capture, callback_args=target))
                    cls = get_message_class(route['type'])
                    publishers.append(rospy.Publisher(source, cls, queue_size=1, latch=route['latch']))
                    inputs[source], outputs[target] = cls(), route
                until(lambda: all(p.get_num_connections() >= 1 for p in publishers), 5, 'input observers')
                # Advance input Header.seq before creating relay publishers. A typed
                # decode/re-publish would renumber it; serialized forwarding must not.
                for unused in range(7):
                    for route, pub in zip(routes, publishers):
                        pub.publish(inputs[route['source']])
                    time.sleep(.03)
                relay_log = open(Path(directory)/'relay.log', 'w')
                try:
                    processes.append(subprocess.Popen(
                        ['python3', '-B', str(here/'relay.py'), '--config', str(here/'config/relay.yaml')],
                        stdout=relay_log, stderr=subprocess.STDOUT))
                    until(lambda: all(p.get_num_connections() >= 2 for p in publishers), 8, 'relay inputs')
                    time.sleep(.3)
                    delays = []
                    for iteration in range(3):
                        if iteration:
                            time.sleep(max(1./r['max_hz'] for r in routes if r.get('max_hz')))
                        for route, pub in zip(routes, publishers):
                            source, target = route['source'], route['target']
                            msg = inputs[source]
                            if hasattr(msg, 'header'):
                                msg.header.frame_id = 'unchanged_frame'
                                msg.header.stamp = rospy.Time(12345, iteration+1)
                            if hasattr(msg, 'child_frame_id'):
                                msg.child_frame_id = 'unchanged_child'
                            if hasattr(msg, 'data'):
                                msg.data = (iteration % 2 == 0 if route['type'] == 'std_msgs/Bool'
                                            else 'ISOLATED_NO_MOTION_%d' % iteration)
                            received.pop(source, None)
                            received.pop(target, None)
                            sent = time.monotonic()
                            pub.publish(msg)
                            until(lambda: source in received and target in received, 4, target)
                            assert received[source][0] == received[target][0], 'Serialized content changed: '+target
                            delays.append(1000*(received[target][1]-sent))
                    assert processes[-1].poll() is None
                    print('FOUR_RELAYS_BYTE_FOR_BYTE_IDENTICAL_PASS (including Header.seq/stamp/frame_id)', flush=True)
                    print('ACCEPTED_MESSAGE_LOCAL_DELAY_MS min=%.3f mean=%.3f max=%.3f' %
                          (min(delays), statistics.mean(delays), max(delays)), flush=True)
                    # Reproduce the original 20 Hz streams on this private master.
                    time.sleep(1.05)
                    history.clear()
                    started, duration = time.monotonic(), 3.2
                    while time.monotonic()-started < duration:
                        for route, pub in zip(routes, publishers):
                            msg = inputs[route['source']]
                            if route['type'] == 'std_msgs/Bool': msg.data = False
                            if route['type'] == 'std_msgs/String': msg.data = 'ISOLATED_REPEATED_STATE'
                            pub.publish(msg)
                        time.sleep(.05)
                    time.sleep(.2)
                    for route in routes:
                        source, target = route['source'], route['target']
                        incoming, outgoing = history.get(source, []), history.get(target, [])
                        assert incoming and outgoing, target
                        originals = {raw for raw,stamp in incoming}
                        assert all(raw in originals for raw,stamp in outgoing), target
                        if route.get('on_change'):
                            assert len(outgoing) == 1, (target, len(outgoing))
                        else:
                            hz = route['max_hz']
                            assert max(2, int(duration*hz*.6)) <= len(outgoing) <= int(duration*hz)+2
                            assert all(b[1]-a[1] >= 1./hz-.035 for a,b in zip(outgoing,outgoing[1:])), target
                        print('%s: input=%d output=%d payload_bytes=%d -> %d' %
                              (target,len(incoming),len(outgoing),
                               sum(len(raw) for raw,stamp in incoming),
                               sum(len(raw) for raw,stamp in outgoing)), flush=True)
                    for route,pub in zip(routes,publishers):
                        if not route.get('on_change'): continue
                        msg = inputs[route['source']]
                        msg.data = True if route['type'] == 'std_msgs/Bool' else 'ISOLATED_CHANGED_STATE'
                        target = route['target']
                        pub.publish(msg)
                        until(lambda: len(history.get(target,[])) == 2, 3, 'changed '+target)
                        late = rospy.wait_for_message(target,get_message_class(route['type']),timeout=3)
                        # rospy may share the existing AnyMsg subscription for
                        # this topic; decode only in the isolated test observer.
                        if isinstance(late, rospy.AnyMsg):
                            late = get_message_class(route['type'])().deserialize(late._buff)
                        assert late.data == msg.data
                    print('RATE_CAP_CHANGE_ONLY_LATCH_PASS', flush=True)
                    print('COMMUNICATION_CHECK_PASS', flush=True)
                finally:
                    relay_log.close()
                    rospy.signal_shutdown('isolated check complete')
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


if __name__ == '__main__':
    main()
