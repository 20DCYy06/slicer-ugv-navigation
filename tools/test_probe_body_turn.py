#!/usr/bin/env python3
"""Pure timing checks; --ros-check OUTPUT is TEST_ONLY in an isolated container."""
import unittest

try:
    import probe_body_turn as probe
except ModuleNotFoundError:
    probe = None


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class ProbeScheduleTest(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(probe, 'the finite body-turn probe is not implemented')

    def test_steady_has_zero_brackets_and_twelve_seconds_same_sign(self):
        for elapsed, expected in ((0., 0.), (1.9999, 0.), (2., .08),
                                  (3.9999, .08), (12., .08), (13.9999, .08),
                                  (14., 0.), (15.9999, 0.), (16., 0.), (20., 0.)):
            with self.subTest(elapsed=elapsed):
                self.assertEqual(probe.command_at('steady', elapsed), expected)

    def test_alternate_reverses_every_two_seconds_without_zero_gaps(self):
        for elapsed, expected in ((1.9999, 0.), (2., .08), (3.9999, .08),
                                  (4., -.08), (5.9999, -.08), (6., .08),
                                  (8., -.08), (10., .08), (12., -.08),
                                  (13.9999, -.08), (14., 0.), (16., 0.)):
            with self.subTest(elapsed=elapsed):
                self.assertEqual(probe.command_at('alternate', elapsed), expected)

    def test_steady_negative_is_the_mirror_of_steady(self):
        for tick in range(321):
            elapsed = tick / 20.
            self.assertEqual(probe.command_at('steady-negative', elapsed),
                             -probe.command_at('steady', elapsed))

    def test_both_sequences_finish_at_sixteen_seconds_and_twenty_hz(self):
        for case in ('steady', 'steady-negative', 'alternate'):
            clock, received = Clock(), []
            probe.run_schedule(case, lambda wz: received.append((clock.now, wz)),
                               clock.monotonic, clock.sleep)
            self.assertEqual(len(received), 321)
            self.assertEqual(received[0], (0., 0.))
            self.assertEqual(received[-1], (16., 0.))
            self.assertEqual(clock.now, 16.)
            self.assertEqual(sum(wz != 0. for _, wz in received), 240)
            for index, (stamp, _) in enumerate(received):
                self.assertAlmostEqual(stamp, index / 20.)

    def test_late_wakeup_uses_elapsed_time_and_does_not_extend_nonzero(self):
        clock, received = Clock(), []

        def late_sleep(duration):
            clock.sleep(duration + (3. if 3.9 < clock.now < 4. else 0.))

        probe.run_schedule('alternate', lambda wz: received.append((clock.now, wz)),
                           clock.monotonic, late_sleep)
        self.assertTrue(any(stamp > 6. for stamp, _ in received))
        self.assertTrue(all(wz == 0. for stamp, wz in received if stamp >= 14.))
        self.assertLessEqual(clock.now, 16.05)
        self.assertTrue(all(b[0] - a[0] >= .05 - 1e-9 for a, b in zip(received, received[1:])),
                        'a delayed wakeup must skip missed ticks rather than burst commands')

    def test_exception_and_ctrl_c_end_with_zero(self):
        for failure in (RuntimeError('TEST_ONLY publish failed'), KeyboardInterrupt()):
            clock, received = Clock(), []

            def publish(wz):
                received.append(wz)
                if wz != 0.:
                    raise failure

            with self.assertRaises(type(failure)):
                probe.run_schedule('steady', publish, clock.monotonic, clock.sleep)
            self.assertEqual(received[-2:], [.08, 0.])

    def test_shutdown_stops_before_next_nonzero_and_finishes_zero(self):
        clock, received = Clock(), []
        probe.run_schedule('steady', lambda wz: received.append(wz),
                           clock.monotonic, clock.sleep, lambda: clock.now >= 3.)
        self.assertEqual(clock.now, 3.)
        self.assertEqual(received[-1], 0.)
        self.assertEqual(sum(wz != 0. for wz in received), 20)

    def test_ready_requires_true_and_a_fresh_received_observation(self):
        self.assertTrue(hasattr(probe, 'ready_fresh'), 'driver ready observation is not implemented')
        for value, received, now, expected in ((True, 10., 10., True),
                                               (True, 10., 10.49, True),
                                               (False, 10., 10., False),
                                               (True, 10., 10.51, False),
                                               (False, float('-inf'), 10., False)):
            with self.subTest(value=value, received=received, now=now):
                self.assertEqual(probe.ready_fresh(value, received, now, .5), expected)

    def test_lost_ready_during_nonzero_ends_with_zero(self):
        self.assertTrue(hasattr(probe, 'ready_fresh'), 'driver ready observation is not implemented')
        clock, received = Clock(), []

        def require_ready():
            if not probe.ready_fresh(clock.now < 2.1, clock.now, clock.now, .5):
                raise RuntimeError('driver ready lost')
            return False

        with self.assertRaisesRegex(RuntimeError, 'driver ready lost'):
            probe.run_schedule('steady', lambda wz: received.append(wz),
                               clock.monotonic, clock.sleep, require_ready)
        self.assertEqual(received[-1], 0.)
        self.assertLess(clock.now, 2.2)

    def test_false_ready_during_run_remains_latched_after_true_recovers(self):
        self.assertTrue(hasattr(probe, 'ready_update'), 'ready false events are not latched')
        observation = probe.ready_update((False, float('-inf'), False), False, 1., False)
        self.assertEqual(observation, (False, 1., False))
        observation = probe.ready_update(observation, True, 2., False)
        self.assertEqual(observation, (True, 2., False))
        observation = probe.ready_update(observation, False, 3., True)
        self.assertEqual(observation, (False, 3., True))
        observation = probe.ready_update(observation, True, 3.01, True)
        self.assertEqual(observation, (True, 3.01, True))


def ros_check(directory, startup_only=False):
    """TEST_ONLY: real TCPROS in a private master; never a hardware result."""
    import json
    import os
    from pathlib import Path
    import signal
    import subprocess
    import sys
    import threading
    import time
    import xmlrpc.client

    if not Path('/.dockerenv').exists() or {p.name for p in Path('/sys/class/net').iterdir()} != {'lo'}:
        raise RuntimeError('TEST_ONLY requires a no-device --network none container')
    output = Path(directory)
    output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:12418'
    os.environ['ROS_IP'] = '127.0.0.1'
    core_log = (output / 'master.log').open('w')
    core = subprocess.Popen(['roscore', '-p', '12418'], stdout=core_log,
                            stderr=subprocess.STDOUT, start_new_session=True)
    child = thread = None
    stop_ready, pause_ready = threading.Event(), threading.Event()
    result = {'TEST_ONLY': True, 'no_hardware': True, 'passed': False, 'cases': []}
    try:
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        deadline = time.monotonic() + 10.
        while True:
            try:
                master.getPid('/TEST_ONLY_probe_check')
                break
            except OSError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.1)
        import rospy
        from geometry_msgs.msg import Twist
        from std_msgs.msg import Bool
        rospy.init_node('TEST_ONLY_probe_sink', disable_signals=True)
        rospy.set_param('/slicer_c30d_driver/provenance/effective_config/telemetry_timeout_s', .5)
        ready_pub = rospy.Publisher('/slicer_c30d_driver/ready', Bool, queue_size=1, latch=True)
        ready_value = [True]

        def ready_source():
            while not stop_ready.is_set():
                if not pause_ready.is_set():
                    ready_pub.publish(Bool(ready_value[0]))
                stop_ready.wait(.05)

        thread = threading.Thread(target=ready_source)
        thread.start()

        def no_external_subscriber():
            nonlocal child
            with (output / 'no-external-subscriber.log').open('w') as log:
                child = subprocess.Popen([sys.executable, 'tools/probe_body_turn.py', 'steady'],
                                         stdout=log, stderr=subprocess.STDOUT)
                try:
                    rc = child.wait(timeout=7.)
                except subprocess.TimeoutExpired as exc:
                    raise AssertionError('the raw self-observer incorrectly satisfied external readiness') from exc
                assert rc == 1, rc
            result['cases'].append({'case': 'no-external-subscriber', 'passed': True})

        if startup_only:
            no_external_subscriber()
            result['passed'] = True
            return
        received = []

        def receive(msg):
            received.append((time.monotonic(), msg.angular.z,
                             [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y],
                             msg._connection_header['callerid']))

        subscriber = rospy.Subscriber('/cmd_vel/nav_raw', Twist, receive, queue_size=100)
        # A silent, registered move_base peer must coexist with the probe: its
        # local costmap is still needed by Guard even when no goal is active.
        peer = rospy.Publisher('/cmd_vel/nav_raw', Twist, queue_size=1)
        for case in ('steady', 'alternate', 'interrupt', 'ready-false', 'ready-stale'):
            before = len(received)
            with (output / (case + '.log')).open('w') as log:
                child = subprocess.Popen([sys.executable, 'tools/probe_body_turn.py',
                                          'alternate' if case in ('alternate', 'interrupt') else 'steady'],
                                         stdout=log, stderr=subprocess.STDOUT)
                deadline = time.monotonic() + 8.
                while not any(row[1] != 0. for row in received[before:]):
                    if child.poll() is not None or time.monotonic() >= deadline:
                        raise AssertionError('probe did not publish nonzero: ' + case)
                    time.sleep(.05)
                state = master.getSystemState('/TEST_ONLY_probe_check')[2]
                emitted = {topic for topic, nodes in state[0]
                           if any('ugv_probe_body_turn_' in node for node in nodes)}
                assert emitted == {'/rosout', '/cmd_vel/nav_raw'}, emitted
                if case == 'interrupt':
                    child.send_signal(signal.SIGINT)
                elif case == 'ready-false':
                    ready_value[0] = False
                elif case == 'ready-stale':
                    pause_ready.set()
                rc = child.wait(timeout=20.)
                expected_rc = 130 if case == 'interrupt' else 1 if case.startswith('ready-') else 0
                assert rc == expected_rc, (case, rc)
                time.sleep(.15)
            ready_value[0] = True
            pause_ready.clear()
            rows = received[before:]
            assert rows and rows[0][1] == rows[-1][1] == 0., case
            assert all(axis == 0. for row in rows for axis in row[2]), case
            assert all(row[1] in (0., .08, -.08) for row in rows), case
            signs = []
            for row in rows:
                if not signs or signs[-1] != row[1]:
                    signs.append(row[1])
            expected = ([0., .08, 0.] if case != 'alternate' else
                        [0., .08, -.08, .08, -.08, .08, -.08, 0.])
            assert signs == expected, (case, signs)
            duration = rows[-1][0] - rows[0][0]
            if case in ('steady', 'alternate'):
                assert 15.8 <= duration <= 16.5, (case, duration)
                assert 310 <= len(rows) <= 325, (case, len(rows))
            elif case.startswith('ready-'):
                assert duration < 2.8, (case, duration)
            (output / (case + '-received.json')).write_text(json.dumps(rows, indent=2) + '\n')
            result['cases'].append({'case': case, 'messages': len(rows),
                                    'duration_s': duration, 'signs': signs, 'passed': True})
        with (output / 'competing-stream.log').open('w') as log:
            before = len(received)
            child = subprocess.Popen([sys.executable, 'tools/probe_body_turn.py', 'steady'],
                                     stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 8.
            while not any(row[1] != 0. for row in received[before:]):
                if child.poll() is not None or time.monotonic() >= deadline:
                    raise AssertionError('silent registered peer prevented probe startup')
                time.sleep(.05)
            conflict_at = time.monotonic()
            peer.publish(Twist())  # Even an actual zero from another source stops the probe.
            assert child.wait(timeout=3.) == 1
            time.sleep(.15)
            own_rows = [row for row in received[before:] if 'ugv_probe_body_turn_' in row[3]]
            assert own_rows[-1][1] == 0.
            assert own_rows[-1][0] - conflict_at < .3
        peer.unregister()
        subscriber.unregister()
        result['cases'].append({'case': 'silent-registered-peer', 'passed': True})
        result['cases'].append({'case': 'actual-competing-stream', 'passed': True})
        no_external_subscriber()
        result['passed'] = True
    finally:
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            child.wait(timeout=5.)
        stop_ready.set()
        if thread is not None:
            thread.join(2.)
        os.killpg(core.pid, signal.SIGINT)
        try:
            core.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            os.killpg(core.pid, signal.SIGKILL)
            core.wait()
        core_log.close()
        (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    import sys
    if len(sys.argv) == 3 and sys.argv[1] in ('--ros-check', '--ros-check-startup'):
        ros_check(sys.argv[2], startup_only=sys.argv[1] == '--ros-check-startup')
    else:
        unittest.main()
