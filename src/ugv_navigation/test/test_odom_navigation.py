#!/usr/bin/env python3
"""TEST_ONLY software navigation contract, never a hardware validation.

Run through odom.test in a no-device, network-none container with its own ROS
master. Only the plant/sensors are synthetic: move_base, GlobalPlanner, TEB,
costmaps, GetPlan and MoveBaseAction are real. This file imports no production
code and never publishes velocity, Action results, or map->odom.

The plant integrates the last raw Twist without a watchdog, collision clamp,
goal snapping, or goal knowledge. Consequently, an absent navigation stop
cannot be hidden by the fixture. The rectangular room and optional wall are
fixed in odom; 720 finite ray intersections are reported in base_link.
"""

import math
import threading
import time
import unittest
from collections import deque

import actionlib
import rospy
import rostest
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest
from sensor_msgs.msg import LaserScan


class TestOnlyDifferentialPlant:
    """TEST_ONLY ideal planar differential kinematics and finite lidar rays."""

    ROOM = (-4.0, 4.0, -1.5, 1.5)
    BEAMS = 720

    def __init__(self, command_topic='/ugv/cmd_vel/nav_raw'):
        self.lock = threading.RLock()
        self.x = self.y = self.yaw = 0.0
        self.velocity = self.omega = 0.0
        self.scan_enabled = True
        self.wall_x = None
        self.obstacle_segments = ()
        self.error = None
        self.commands = deque(maxlen=30000)
        self.poses = deque(maxlen=30000)
        self.maps = {}
        self.stop_event = threading.Event()
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=5)
        # TEST_ONLY source: this ideal plant supplies both odometry interfaces.
        # It is not a LIO/EKF model or evidence of real fusion performance.
        self.lio_pub = rospy.Publisher('/lio/odom', Odometry, queue_size=5)
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=5)
        self.tf = tf2_ros.TransformBroadcaster()
        self.command_sub = rospy.Subscriber(
            command_topic, Twist, self._command, queue_size=100)
        self.map_subs = [
            rospy.Subscriber('/ugv/move_base/{}/costmap'.format(name),
                             OccupancyGrid, self._map, callback_args=name,
                             queue_size=1)
            for name in ('global_costmap', 'local_costmap')
        ]
        self.thread = threading.Thread(target=self._run,
                                       name='TEST_ONLY_differential_plant')
        self.thread.daemon = True
        self.thread.start()

    def _command(self, message):
        values = (message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z)
        with self.lock:
            if not all(math.isfinite(value) for value in values):
                self.error = 'Navigation emitted a non-finite Twist: {!r}'.format(values)
                return
            self.commands.append((time.monotonic(),) + values)
            self.velocity, self.omega = values[0], values[5]

    def _map(self, message, name):
        with self.lock:
            self.maps[name] = message

    def pose(self):
        with self.lock:
            return self.x, self.y, self.yaw

    def commands_since(self, since):
        with self.lock:
            return [entry for entry in self.commands if entry[0] >= since]

    def poses_since(self, since):
        with self.lock:
            return [entry for entry in self.poses if entry[0] >= since]

    def set_scan(self, enabled):
        with self.lock:
            self.scan_enabled = enabled

    def set_wall(self, wall_x):
        with self.lock:
            self.wall_x = wall_x

    def set_segments(self, segments):
        """Fixed odom-frame finite lidar geometry; never a collision clamp."""
        checked = []
        for a, b in segments:
            values = tuple(float(value) for value in (*a, *b))
            if len(values) != 4 or not all(math.isfinite(value) for value in values):
                raise ValueError('A finite segment needs two finite planar endpoints')
            if values[:2] == values[2:]:
                raise ValueError('A finite segment needs distinct endpoints')
            checked.append((values[:2], values[2:]))
        with self.lock:
            self.obstacle_segments = tuple(checked)

    def grid(self, name):
        with self.lock:
            return self.maps.get(name)

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        self.command_sub.unregister()
        for subscriber in self.map_subs:
            subscriber.unregister()
        self.odom_pub.unregister()
        self.lio_pub.unregister()
        self.scan_pub.unregister()

    def _run(self):
        previous = time.monotonic()
        next_sensor = previous
        try:
            while not self.stop_event.is_set() and not rospy.is_shutdown():
                now = time.monotonic()
                elapsed, previous = now - previous, now
                with self.lock:
                    # Exact constant-Twist integration, with no collision response.
                    delta_yaw = self.omega * elapsed
                    if abs(self.omega) > 1.0e-9:
                        radius = self.velocity / self.omega
                        self.x += radius * (math.sin(self.yaw + delta_yaw)
                                            - math.sin(self.yaw))
                        self.y -= radius * (math.cos(self.yaw + delta_yaw)
                                            - math.cos(self.yaw))
                    else:
                        self.x += self.velocity * math.cos(self.yaw) * elapsed
                        self.y += self.velocity * math.sin(self.yaw) * elapsed
                    self.yaw = math.atan2(math.sin(self.yaw + delta_yaw),
                                          math.cos(self.yaw + delta_yaw))
                    state = (self.x, self.y, self.yaw,
                             self.velocity, self.omega)
                    self.poses.append((now,) + state[:3])
                    scan_enabled, wall_x = self.scan_enabled, self.wall_x
                    obstacle_segments = self.obstacle_segments
                if now >= next_sensor:
                    self._publish(state, scan_enabled, wall_x, obstacle_segments)
                    next_sensor = now + 0.05
                self.stop_event.wait(0.01)
        except Exception as error:  # Surface a publisher/thread failure in unittest.
            with self.lock:
                self.error = repr(error)

    def _publish(self, state, scan_enabled, wall_x, obstacle_segments=()):
        x, y, yaw, velocity, omega = state
        stamp = rospy.Time.now()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf.sendTransform(transform)

        odom = Odometry()
        odom.header = transform.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation = transform.transform.rotation
        odom.pose.covariance[0] = odom.pose.covariance[7] = 0.001
        odom.pose.covariance[35] = 0.001
        odom.twist.twist.linear.x = velocity
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)
        self.lio_pub.publish(odom)

        if not scan_enabled:
            return
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / self.BEAMS
        scan.angle_max = scan.angle_min + (self.BEAMS - 1) * scan.angle_increment
        scan.scan_time = 0.05
        scan.time_increment = 0.0  # An instantaneous ideal scan.
        scan.range_min, scan.range_max = 0.05, 3.5
        # Finite sensor range leaves reachable unknown cells in front of the
        # robot, without an intervening wall. Max-range rays exceed the 3m
        # marking limit and clear only, avoiding an artificial obstacle ring.
        # laser_geometry excludes returns >= range_max. A finite max-range
        # return must stay just inside it (as inf_is_valid does upstream).
        scan.ranges = [min(scan.range_max - 0.001, self._ray(x, y, yaw + scan.angle_min
                                + index * scan.angle_increment, wall_x, obstacle_segments))
                       for index in range(self.BEAMS)]
        self.scan_pub.publish(scan)

    @classmethod
    def _ray(cls, x, y, angle, wall_x, obstacle_segments=()):
        dx, dy = math.cos(angle), math.sin(angle)
        xmin, xmax, ymin, ymax = cls.ROOM
        distances = []
        if abs(dx) > 1.0e-10:
            for boundary in (xmin, xmax):
                distance = (boundary - x) / dx
                hit_y = y + distance * dy
                if distance > 0.0 and ymin <= hit_y <= ymax:
                    distances.append(distance)
            if wall_x is not None:
                distance = (wall_x - x) / dx
                hit_y = y + distance * dy
                if distance > 0.0 and ymin <= hit_y <= ymax:
                    distances.append(distance)
        if abs(dy) > 1.0e-10:
            for boundary in (ymin, ymax):
                distance = (boundary - y) / dy
                hit_x = x + distance * dx
                if distance > 0.0 and xmin <= hit_x <= xmax:
                    distances.append(distance)
        for a, b in obstacle_segments:
            sx, sy = b[0] - a[0], b[1] - a[1]
            denominator = dx * sy - dy * sx
            ax, ay = a[0] - x, a[1] - y
            if abs(denominator) <= 1.0e-10:
                if abs(ax * dy - ay * dx) <= 1.0e-10:
                    projections = (ax * dx + ay * dy,
                                   (b[0] - x) * dx + (b[1] - y) * dy)
                    if max(projections) >= 0.0:
                        distances.append(max(0.0, min(projections)))
                continue
            distance = (ax * sy - ay * sx) / denominator
            fraction = (ax * dy - ay * dx) / denominator
            if distance > 0.0 and -1.0e-10 <= fraction <= 1.0 + 1.0e-10:
                distances.append(distance)
        if not distances:
            raise RuntimeError('TEST_ONLY plant left the finite ray-casting room')
        return max(0.05, min(distances))


class OdomNavigationTest(unittest.TestCase):
    """Breaks caught: unsafe routing, inert/fake navigation, cancel revival,
    stale-sensor driving, missing goal continuation, and traversable walls.
    """

    @classmethod
    def setUpClass(cls):
        if rospy.get_param('/use_sim_time', False):
            raise AssertionError('This fixture requires live wall time')
        cls.plant = TestOnlyDifferentialPlant()
        cls.client = actionlib.SimpleActionClient('/ugv/move_base', MoveBaseAction)
        if not cls.client.wait_for_server(rospy.Duration(15.0)):
            cls.plant.close()
            raise AssertionError('Real /ugv/move_base Action server missing after 15s')
        rospy.wait_for_service('/ugv/move_base/make_plan', timeout=10.0)
        cls.make_plan = rospy.ServiceProxy('/ugv/move_base/make_plan', GetPlan)

    @classmethod
    def tearDownClass(cls):
        cls.client.cancel_all_goals()
        cls.plant.close()

    def setUp(self):
        self.plant.set_scan(True)
        self._wait(lambda: self.plant.grid('global_costmap') is not None
                   and self.plant.grid('local_costmap') is not None,
                   12.0, 'Both real rolling costmaps must publish')
        self.assertEqual(
            rospy.get_param('/ugv/move_base/base_local_planner'),
            'teb_local_planner/TebLocalPlannerROS')

    def tearDown(self):
        self.plant.set_scan(True)
        self.client.cancel_all_goals()
        # Navigation must deliver the stop; the plant does not inject one.
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            with self.plant.lock:
                stopped = abs(self.plant.velocity) < 1.0e-6 and abs(self.plant.omega) < 1.0e-6
            if stopped:
                break
            time.sleep(0.02)

    def _wait(self, predicate, timeout, message):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.assertIsNone(self.plant.error, self.plant.error)
            if predicate():
                return
            time.sleep(0.025)
        self.fail('{} (timeout {:.1f}s; pose={!r}; action={})'.format(
            message, timeout, self.plant.pose(), self.client.get_state()))

    @staticmethod
    def _pose_message(x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.w = 1.0
        return pose

    def _plan(self, x, y):
        current = self.plant.pose()
        request = GetPlanRequest()
        request.start = self._pose_message(current[0], current[1])
        request.goal = self._pose_message(x, y)
        request.tolerance = 0.0
        outcome = {}
        completed = threading.Event()

        def call():
            try:
                outcome['response'] = self.make_plan(request)
            except Exception as error:
                outcome['error'] = error
            finally:
                completed.set()

        thread = threading.Thread(target=call, name='TEST_ONLY_GetPlan_timeout')
        thread.daemon = True
        thread.start()
        self.assertTrue(completed.wait(5.0), 'Real GetPlan service exceeded 5s')
        self.assertNotIn('error', outcome, repr(outcome.get('error')))
        return outcome['response'].plan.poses

    def _send(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose = self._pose_message(x, y)
        self.client.send_goal(goal)

    @staticmethod
    def _moving(command):
        return abs(command[1]) > 0.001 or abs(command[6]) > 0.001

    def _require_motion(self, since, start):
        self._wait(
            lambda: any(self._moving(command)
                        for command in self.plant.commands_since(since))
            and math.hypot(self.plant.pose()[0] - start[0],
                           self.plant.pose()[1] - start[1]) >= 0.03,
            10.0, 'Real TEB must issue nonzero raw Twist and move the plant')

    def _require_success(self, target, timeout=25.0):
        self.assertTrue(self.client.wait_for_result(rospy.Duration(timeout)),
                        'Original MoveBaseAction exceeded {:.1f}s'.format(timeout))
        self.assertEqual(self.client.get_state(), GoalStatus.SUCCEEDED,
                         self.client.get_goal_status_text())
        pose = self.plant.pose()
        # Production tolerances are .15m/.25rad; .02 absorbs sampling only.
        self.assertLessEqual(math.hypot(pose[0] - target[0], pose[1] - target[1]), 0.17)
        self.assertLessEqual(abs(pose[2]), 0.27)

    def _require_still(self, since, start, duration):
        time.sleep(duration)
        commands = self.plant.commands_since(since)
        self.assertFalse(any(self._moving(command) for command in commands),
                         'Navigation emitted nonzero raw Twist while it must stop')
        samples = self.plant.poses_since(since)
        self.assertTrue(samples, 'No kinematic samples in stop observation interval')
        self.assertLessEqual(max(math.hypot(entry[1] - start[0], entry[2] - start[1])
                                 for entry in samples), 0.01)
        self.assertLessEqual(max(abs(math.atan2(math.sin(entry[3] - start[2]),
                                               math.cos(entry[3] - start[2])))
                                 for entry in samples), 0.02)

    @staticmethod
    def _cell(grid, x, y):
        column = math.floor((x - grid.info.origin.position.x) / grid.info.resolution)
        row = math.floor((y - grid.info.origin.position.y) / grid.info.resolution)
        if not (0 <= column < grid.info.width and 0 <= row < grid.info.height):
            return None
        return grid.data[row * grid.info.width + column]

    def test_01_observed_plan_and_unknown_or_outside_window_rejection(self):
        # A permissive allow_unknown or static/oversized global map breaks this.
        x, y, _ = self.plant.pose()
        self._wait(lambda: self._cell(self.plant.grid('global_costmap'), x + 0.7, y) == 0,
                   8.0, 'Forward target must first be observed as free')
        plan = self._plan(x + 0.7, y)
        self.assertGreater(len(plan), 2, 'Observed free target has no actual global path')
        self.assertLess(math.hypot(plan[-1].pose.position.x - (x + 0.7),
                                   plan[-1].pose.position.y - y), 0.1)
        grid = self.plant.grid('global_costmap')
        self.assertIsNone(self._cell(grid, x + 4.5, y), 'Outside-window fixture is invalid')
        self.assertEqual(self._plan(x + 4.5, y), [], 'Outside rolling window was planned')
        self.assertEqual(self._cell(grid, x + 3.7, y), -1,
                         'In-window target beyond lidar range must remain unobserved')
        self.assertEqual(self._plan(x + 3.7, y), [], 'Unknown space was accepted as traversable')

    def test_02_real_teb_motion_and_action_success(self):
        # Fake SUCCEEDED without real control, or a disconnected raw output, fails.
        start, since = self.plant.pose(), time.monotonic()
        target = (start[0] + 0.7, start[1])
        self._send(*target)
        self._require_motion(since, start)
        self._require_success(target)
        commands = self.plant.commands_since(since)
        self.assertTrue(any(command[1] > 0.01 for command in commands))
        self.assertLessEqual(max(abs(command[1]) for command in commands), 0.135)
        self.assertLessEqual(max(abs(command[6]) for command in commands), 0.27)
        self.assertTrue(all(max(abs(value) for value in command[2:6]) < 1.0e-6
                            for command in commands), 'Differential command has lateral/3D motion')

    def test_03_cancel_is_preempted_and_does_not_revive(self):
        # A goal resubmitter or missing stop on cancel breaks this observation.
        start, since = self.plant.pose(), time.monotonic()
        self._send(start[0] + 0.8, start[1])
        self._require_motion(since, start)
        self.client.cancel_goal()
        self.assertTrue(self.client.wait_for_result(rospy.Duration(5.0)),
                        'Cancel did not terminate MoveBaseAction in 5s')
        self.assertEqual(self.client.get_state(), GoalStatus.PREEMPTED)
        time.sleep(0.3)  # Allow the in-flight transport/controller sample to drain.
        stopped_at, stopped_pose = time.monotonic(), self.plant.pose()
        self._require_still(stopped_at, stopped_pose, 2.0)
        self.assertEqual(self.client.get_state(), GoalStatus.PREEMPTED,
                         'Canceled goal revived without a new goal')

    def test_04_scan_loss_stops_then_original_goal_continues(self):
        # Infinite sensor freshness or treating transient loss as terminal fails.
        start, since = self.plant.pose(), time.monotonic()
        target = (start[0] + 0.7, start[1])
        self._send(*target)
        self._require_motion(since, start)
        self.plant.set_scan(False)
        try:
            time.sleep(1.5)  # Longer than .5s observation freshness + control cycles.
            stopped_at, stopped_pose = time.monotonic(), self.plant.pose()
            self._require_still(stopped_at, stopped_pose, 1.0)
            self.assertEqual(self.client.get_state(), GoalStatus.ACTIVE,
                             'A 2.5s sensor outage must retain the original goal')
        finally:
            self.plant.set_scan(True)
        # No send_goal here: recovery must continue the exact original Action.
        self._require_success(target)

    def test_05_full_width_wall_blocks_path_and_is_never_crossed(self):
        # Ignored obstacle marking, unknown bypass, or unsafe driving fails.
        start = self.plant.pose()
        wall_x = start[0] + 0.9
        self.plant.set_wall(wall_x)
        self._wait(lambda: self._cell(self.plant.grid('global_costmap'), wall_x, 0.0)
                   is not None and self._cell(self.plant.grid('global_costmap'),
                                             wall_x, 0.0) >= 65,
                   8.0, 'Real scan obstacle marking must observe the full-width wall')
        target = (start[0] + 1.7, start[1])
        self.assertEqual(self._plan(*target), [], 'Global path crossed the full-width wall')
        since = time.monotonic()
        self._send(*target)
        self.assertTrue(self.client.wait_for_result(rospy.Duration(20.0)),
                        'Unreachable wall goal did not terminate in 20s')
        self.assertEqual(self.client.get_state(), GoalStatus.ABORTED,
                         self.client.get_goal_status_text())
        samples = self.plant.poses_since(since)
        self.assertTrue(samples)
        # Debug envelope from the declared conservative Slicer footprint, not a
        # claim of measured dimensions. Check every integrated sample, not just
        # the final pose, and do not clamp the plant at the obstacle.
        footprint = ((-0.165, -0.14), (-0.165, 0.14),
                     (0.27, 0.14), (0.27, -0.14))
        foremost = max(entry[1] + fx * math.cos(entry[3]) - fy * math.sin(entry[3])
                       for entry in samples for fx, fy in footprint)
        self.assertLessEqual(foremost, wall_x + 0.02,
                             'Integrated robot footprint crossed the physical wall')


if __name__ == '__main__':
    rospy.init_node('TEST_ONLY_odom_navigation', anonymous=True)
    rostest.rosrun('ugv_navigation', 'odom_navigation', OdomNavigationTest)
