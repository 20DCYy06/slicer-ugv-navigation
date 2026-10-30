#!/usr/bin/env python3
"""TEST_ONLY real TEB -> real Guard -> real Mux -> ideal differential model.

The model executes final Twist, never raw. It has no PWM, motor deadband,
inertia or goal knowledge. Driver/serial behavior is tested separately.
"""
import json
import math
import threading
import time
import unittest

import actionlib
import rospy
import rostest
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from test_odom_navigation import TestOnlyDifferentialPlant


class GuardedNavigationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lock = threading.Lock()
        cls.state = None
        cls.raw = []
        cls.guarded = []
        cls.plant = TestOnlyDifferentialPlant('/TEST_ONLY_chain/final')
        cls.subs = [
            rospy.Subscriber('/cup_nav/manual_state', String, cls.on_state, queue_size=20),
            rospy.Subscriber('/TEST_ONLY_chain/raw', Twist, cls.on_raw, queue_size=100),
            rospy.Subscriber('/cmd_vel/nav', Twist, cls.on_guarded, queue_size=100),
        ]
        cls.intent = rospy.Publisher('/cup_nav/manual_intent', String, queue_size=1, latch=False)
        cls.client = actionlib.SimpleActionClient('/ugv/move_base', MoveBaseAction)
        if not cls.client.wait_for_server(rospy.Duration(20.0)):
            raise AssertionError('Standard move_base Action server unavailable')

    @classmethod
    def tearDownClass(cls):
        cls.client.cancel_all_goals()
        cls.plant.close()
        for sub in cls.subs:
            sub.unregister()

    @classmethod
    def on_state(cls, msg):
        with cls.lock:
            cls.state = json.loads(msg.data)

    @classmethod
    def on_raw(cls, msg):
        with cls.lock:
            cls.raw.append((time.monotonic(), msg.linear.x, msg.angular.z))

    @classmethod
    def on_guarded(cls, msg):
        with cls.lock:
            cls.guarded.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def wait(self, predicate, timeout, reason):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.assertIsNone(self.plant.error)
            if predicate():
                return
            time.sleep(.025)
        self.fail(reason + '; pose=' + repr(self.plant.pose()))

    def send(self, x, y, yaw=0.):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'odom'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = x, y
        goal.target_pose.pose.orientation.z = math.sin(yaw/2)
        goal.target_pose.pose.orientation.w = math.cos(yaw/2)
        self.client.send_goal(goal)

    def test_01_startup_hold_blocks_real_teb_output(self):
        self.wait(lambda: self.state is not None and
                  self.plant.grid('local_costmap') is not None, 15., 'Missing live context')
        self.assertTrue(self.state['manual_hold'])
        start, since = self.plant.pose(), time.monotonic()
        self.send(start[0] + .5, start[1])
        try:
            self.wait(lambda: any(t >= since and abs(v) > .01 for t, v, w in self.raw),
                      10., 'TEB produced no genuine forward request')
            time.sleep(.5)
            self.assertTrue(all(abs(c[1]) < 1e-9 and abs(c[6]) < 1e-9
                                for c in self.plant.commands_since(since)))
            self.assertLess(math.hypot(self.plant.pose()[0]-start[0],
                                       self.plant.pose()[1]-start[1]), .001)
        finally:
            self.client.cancel_goal()
        self.assertTrue(self.client.wait_for_result(rospy.Duration(5.)))
        self.assertEqual(self.client.get_state(), GoalStatus.PREEMPTED)

    def test_02_explicit_release_forward_arrival_and_cancel(self):
        self.wait(lambda: self.intent.get_num_connections() > 0 and self.state is not None,
                  5., 'Mux handoff transport unavailable')
        request = dict(schema_version=1, intent_id='TEST_ONLY_release', action='release',
                       mux_instance_id=self.state['mux_instance_id'],
                       takeover_generation=self.state['takeover_generation'])
        self.intent.publish(String(data=json.dumps(request)))
        self.wait(lambda: self.state['last_intent_id'] == request['intent_id'] and
                  self.state['last_intent_accepted'] and not self.state['manual_hold'],
                  5., 'Explicit test-only release not acknowledged')
        start, since = self.plant.pose(), time.monotonic()
        target = (start[0]+.5, start[1])
        self.send(*target)
        try:
            self.wait(lambda: any(c[1] > .01 for c in self.plant.commands_since(since)),
                      10., 'Final Mux output did not drive the model')
            self.assertTrue(self.client.wait_for_result(rospy.Duration(25.)))
            self.assertEqual(self.client.get_state(), GoalStatus.SUCCEEDED,
                             self.client.get_goal_status_text())
            self.assertLessEqual(math.hypot(self.plant.pose()[0]-target[0],
                                            self.plant.pose()[1]-target[1]), .17)
            self.assertTrue(any(t >= since and v > .01 for t, v, w in self.guarded))
            commands = self.plant.commands_since(since)
            self.assertTrue(all(0 <= c[1] <= .125001 and abs(c[6]) <= .250001
                                for c in commands))
            self.wait(lambda: abs(self.plant.velocity) < 1e-9 and
                      abs(self.plant.omega) < 1e-9, 3., 'Arrival zero missing downstream')

            self.send(self.plant.pose()[0]+.8, self.plant.pose()[1])
            self.wait(lambda: self.plant.velocity > .01, 10., 'Second goal never moved')
            self.client.cancel_goal()
            self.assertTrue(self.client.wait_for_result(rospy.Duration(5.)))
            self.assertEqual(self.client.get_state(), GoalStatus.PREEMPTED)
            self.wait(lambda: abs(self.plant.velocity) < 1e-9 and
                      abs(self.plant.omega) < 1e-9, 3., 'Cancel zero missing downstream')
            stopped, pose = time.monotonic(), self.plant.pose()
            time.sleep(1.)
            self.assertTrue(all(abs(c[1]) < 1e-9 and abs(c[6]) < 1e-9
                                for c in self.plant.commands_since(stopped)))
            self.assertLess(math.hypot(self.plant.pose()[0]-pose[0],
                                       self.plant.pose()[1]-pose[1]), .001)
        finally:
            self.client.cancel_goal()


if __name__ == '__main__':
    rospy.init_node('TEST_ONLY_chain_operator')
    rostest.rosrun('ugv_navigation', 'guarded_navigation', GuardedNavigationTest)
