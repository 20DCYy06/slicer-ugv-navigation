#!/usr/bin/env python3
"""TEST_ONLY real Guard transport; sensors/TF are synthetic, with no driver.

The same checks run with compensation disabled and enabled. Observations are
the real Guard outputs and hold reasons, never a duplicate Guard algorithm.
"""
from collections import deque
import math
import threading
import time
import unittest

import rospy
import rostest
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String


class GuardTransportTests(unittest.TestCase):
    NODES = ('TEST_ONLY_guard', 'TEST_ONLY_guard_boosted')

    def setUp(self):
        self.lock = threading.RLock()
        self.outputs = {name: deque(maxlen=2000) for name in self.NODES}
        self.reasons = {name: None for name in self.NODES}
        self.sent = {kind: deque(maxlen=200) for kind in ('command', 'grid', 'tf', 'lio')}
        self.sent_count = dict.fromkeys(self.sent, 0)
        self.command = (0., 0.)
        self.pose = (0., 0., 0.)
        self.cell = 0
        self.cell_index = 40 * 80 + 40
        self.command_enabled = self.tf_enabled = self.grid_enabled = True
        self.error = None
        self.stop_event = threading.Event()
        self.raw = rospy.Publisher('/TEST_ONLY_guard/raw', Twist, queue_size=1)
        self.grid = rospy.Publisher('/TEST_ONLY_guard/grid', OccupancyGrid, queue_size=1)
        self.lio = rospy.Publisher('/TEST_ONLY_guard/lio', Odometry, queue_size=1)
        self.broadcaster = tf.TransformBroadcaster()
        self.subscribers = []
        for name in self.NODES:
            self.subscribers.extend((
                rospy.Subscriber('/' + name + '/output', Twist, self._output,
                                 callback_args=name, queue_size=100),
                rospy.Subscriber('/' + name + '/hold_reason', String, self._reason,
                                 callback_args=name, queue_size=10)))
        self.thread = threading.Thread(target=self._publish, daemon=True)
        self.thread.start()
        try:
            self._wait(lambda: self.raw.get_num_connections() == 2
                       and self.grid.get_num_connections() == 2
                       and self.lio.get_num_connections() == 2
                       and all(self.outputs[name] for name in self.NODES), 8.)
            # Outputs arriving while publishers reconnect can belong to the previous
            # test. Require a newly sent stop and new output samples before proceeding.
            self._moving(0., 0.)
        except Exception:
            # unittest skips tearDown when setUp fails. Do not leave this test's
            # producers alive to contaminate the following test's observations.
            self.tearDown()
            raise

    def tearDown(self):
        with self.lock:
            self.command = (0., 0.)
            self.command_enabled = True
        time.sleep(.1)
        self.stop_event.set()
        self.thread.join(2.)
        for subscriber in self.subscribers:
            subscriber.unregister()
        self.raw.unregister()
        self.grid.unregister()
        self.lio.unregister()

    def _output(self, message, name):
        with self.lock:
            self.outputs[name].append((time.monotonic(), message.linear.x, message.angular.z))

    def _reason(self, message, name):
        with self.lock:
            self.reasons[name] = message.data

    def _publish(self):
        try:
            while not self.stop_event.is_set() and not rospy.is_shutdown():
                with self.lock:
                    command, pose, cell = self.command, self.pose, self.cell
                    cell_index = self.cell_index
                    command_enabled = self.command_enabled
                    grid_enabled, tf_enabled = self.grid_enabled, self.tf_enabled
                stamp = rospy.Time.now()
                # Independent LIO input stays live while each existing geometry
                # or command expiry is tested. The dedicated stale-LIO test
                # controls LIO independently of all these inputs.
                lio = Odometry()
                lio.header.stamp = stamp
                lio.header.frame_id = 'TEST_ONLY_guard_grid'
                lio.child_frame_id = 'TEST_ONLY_guard_base'
                lio.pose.pose.orientation.w = 1.
                self.lio.publish(lio)
                with self.lock:
                    self.sent['lio'].append((time.monotonic(), stamp.to_sec()))
                    self.sent_count['lio'] += 1
                if tf_enabled:
                    self.broadcaster.sendTransform(
                        (pose[0], pose[1], 0.), tf.transformations.quaternion_from_euler(0., 0., pose[2]),
                        stamp, 'TEST_ONLY_guard_base', 'TEST_ONLY_guard_grid')
                    with self.lock:
                        self.sent['tf'].append((time.monotonic(), pose))
                        self.sent_count['tf'] += 1
                if grid_enabled:
                    grid = OccupancyGrid()
                    grid.header.frame_id = 'TEST_ONLY_guard_grid'
                    grid.header.stamp = stamp
                    grid.info.resolution = .05
                    grid.info.width = grid.info.height = 80
                    grid.info.origin.position.x = grid.info.origin.position.y = -2.
                    grid.info.origin.orientation.w = 1.
                    grid.data = [0] * 6400
                    grid.data[cell_index] = cell
                    self.grid.publish(grid)
                    with self.lock:
                        self.sent['grid'].append((time.monotonic(), cell_index, cell))
                        self.sent_count['grid'] += 1
                if command_enabled:
                    message = Twist()
                    message.linear.x, message.angular.z = command
                    self.raw.publish(message)
                    with self.lock:
                        self.sent['command'].append((time.monotonic(), command))
                        self.sent_count['command'] += 1
                self.stop_event.wait(.03)
        except Exception as error:
            self.error = repr(error)

    def _wait(self, predicate, timeout=3.):
        started = (time.monotonic(), time.time(), rospy.Time.now().to_sec())
        deadline = started[0] + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.assertIsNone(self.error, self.error)
            with self.lock:
                if predicate():
                    return
            time.sleep(.015)
        with self.lock:
            self.fail('Guard observation timed out: reasons={!r}; last_outputs={!r}; fixture={!r}'.format(
                self.reasons, {name: list(samples)[-3:] for name, samples in self.outputs.items()},
                self._diagnostics(started)))

    def _diagnostics(self, started=None):
        clocks = (time.monotonic(), time.time(), rospy.Time.now().to_sec())
        with self.lock:
            recent = {kind: list(samples)[-3:] for kind, samples in self.sent.items()}
            return dict(test=self._testMethodName, clocks=clocks,
                        monotonic_minus_wall=clocks[0] - clocks[1],
                        monotonic_minus_ros=clocks[0] - clocks[2],
                        elapsed=None if started is None else tuple(a - b for a, b in zip(clocks, started)),
                        raw_connections=self.raw.get_num_connections(),
                        grid_connections=self.grid.get_num_connections(),
                        lio_connections=self.lio.get_num_connections(),
                        publisher_alive=self.thread.is_alive(),
                        enabled=(self.command_enabled, self.grid_enabled, self.tf_enabled),
                        send_count=dict(self.sent_count), recent_sends=recent,
                        last_send_age={kind: clocks[0] - samples[-1][0] if samples else None
                                       for kind, samples in recent.items()},
                        send_intervals={kind: [b[0] - a[0] for a, b in zip(samples, samples[1:])]
                                        for kind, samples in recent.items()})

    def _moving(self, velocity=.06, omega=.05):
        with self.lock:
            requested_at = time.monotonic()
            before = dict(self.sent_count)
            self.command = (velocity, omega)

        def ready():
            if (self.raw.get_num_connections() != 2 or self.grid.get_num_connections() != 2
                    or self.lio.get_num_connections() != 2):
                return False
            # Publishing three fresh samples proves that this round's input
            # producer has run; a connection count or old matching output cannot.
            if any(self.sent_count[kind] < count + 3 for kind, count in before.items()):
                return False
            sent_at = next(sample[0] for sample in self.sent['command']
                           if sample[0] >= requested_at)
            for name in self.NODES:
                samples = list(self.outputs[name])[-3:]
                if (self.reasons[name] != '' or len(samples) < 3
                        or samples[0][0] < max(requested_at, sent_at)
                        or samples[-1][0] - samples[0][0] < .075
                        or any(abs(sample[1] - velocity) >= .001
                               or abs(sample[2] - omega) >= .001 for sample in samples)):
                    return False
            return True

        self._wait(ready)
        rospy.loginfo('Guard fixture motion ready: %s', self._diagnostics())

    def _held(self, reason, timeout=3.):
        self._wait(lambda: all(self.reasons[name] is not None
                               and reason in self.reasons[name]
                               and self.outputs[name][-1][1:] == (0., 0.)
                               for name in self.NODES), timeout)
        since = time.monotonic()
        time.sleep(.16)
        with self.lock:
            for name in self.NODES:
                samples = [entry[1:] for entry in self.outputs[name] if entry[0] >= since]
                self.assertGreaterEqual(len(samples), 2, name)
                self.assertTrue(all(sample == (0., 0.) for sample in samples),
                                '{} retained motion during {}: {!r}'.format(name, reason, samples))

    def test_01_forbidden_reverse_arc_and_tiny_reverse_stop_with_reason(self):
        for velocity, omega in ((-.02, .25), (-1.e-12, -.05)):
            with self.subTest(vx=velocity, wz=omega):
                self._moving(0., .25)
                with self.lock:
                    self.command = (velocity, omega)
                # In particular, the enabled compensation must not resurrect a spin.
                self._held('negative body vx', timeout=1.)

    def test_02_forward_pure_turn_and_explicit_zero_remain_legal(self):
        self._moving()
        self._moving(0., .25)
        self._moving(0., -.25)
        with self.lock:
            self.command = (0., 0.)
        self._wait(lambda: all(self.outputs[name][-1][1:] == (0., 0.)
                               and self.reasons[name] == '' for name in self.NODES), .4)

    def test_03_expired_command_stops_despite_fresh_geometry(self):
        self._moving()
        with self.lock:
            self.command_enabled = False
        self._held('command missing/stale', timeout=1.)

    def test_04_expired_tf_stops_despite_fresh_command_and_grid(self):
        self._moving()
        with self.lock:
            self.tf_enabled = False
        self._held('fresh base TF', timeout=1.)

    def test_05_expired_grid_stops_despite_fresh_command_and_tf(self):
        self._moving()
        with self.lock:
            self.grid_enabled = False
        self._held('fresh local grid', timeout=2.2)
        # An explicit stop must still propagate without requiring fresh geometry.
        with self.lock:
            self.command = (0., 0.)
        self._wait(lambda: all(self.outputs[name][-1][1:] == (0., 0.)
                               and self.reasons[name] == '' for name in self.NODES), .4)

    def test_06_unknown_current_footprint_stops_both_turn_policies(self):
        self._moving(0., .25)
        with self.lock:
            self.cell = -1
        self._held('swept footprint')

    def test_07_occupied_current_footprint_stops_both_turn_policies(self):
        self._moving(0., .25)
        with self.lock:
            self.cell = 100
        self._held('swept footprint')

    def test_08_outside_footprint_stops_with_fresh_context(self):
        self._moving(0., .25)
        with self.lock:
            self.pose = (1.9, 0., 0.)
        self._held('swept footprint')

    def test_09_non_finite_command_stops(self):
        self._moving()
        with self.lock:
            self.command = (math.nan, .25)
        self._held('non-finite input')

    def test_10_future_forward_sweep_stops_before_current_footprint_contact(self):
        self._moving(.1, 0.)
        with self.lock:
            self.cell_index = 40 * 80 + 47  # x=[.35,.40], beyond current front x=.27.
            self.cell = 100
        self._held('swept footprint')
        with self.lock:
            self.cell = 0
        self._moving(.1, 0.)  # Fresh clear context permits the next allowed request.

    def test_11_future_rotation_sweep_checks_the_footprint_corners(self):
        self._moving(0., .25)
        with self.lock:
            # x=[.20,.25], y=[.15,.20]: currently clear, hit by the rotating front corner.
            self.cell_index = 43 * 80 + 44
            self.cell = 100
        self._held('swept footprint')


if __name__ == '__main__':
    rospy.init_node('TEST_ONLY_guard_transport')
    rostest.rosrun('ugv_slicer_control', 'guard_transport', GuardTransportTests)
