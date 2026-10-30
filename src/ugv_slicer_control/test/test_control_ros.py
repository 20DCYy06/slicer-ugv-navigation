#!/usr/bin/env python3
"""Real ROS transport checks against the verbatim mux; no driver or velocity input."""
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import unittest

import rospy
import rostest
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from std_msgs.msg import Bool, String


class ControlTransportTests(unittest.TestCase):
    def setUp(self):
        self.statuses, self.cancellations, self.states = [GoalStatus(status=1)], [], []
        self.status = rospy.Publisher('/ugv/move_base/status', GoalStatusArray, queue_size=1, latch=True)
        self.cancel = rospy.Subscriber('/ugv/move_base/cancel', GoalID, self.receive_cancel, queue_size=10)
        self.state = rospy.Subscriber('/cup_nav/manual_state', String,
                                      lambda message: self.states.append(json.loads(message.data)), queue_size=20)
        # A rosbag/rostopic observer receives intent but cannot acknowledge or execute it.
        self.intent_observer = rospy.Subscriber('/cup_nav/manual_intent', String,
                                               lambda message: None, queue_size=10)
        self.running = True
        self.thread = threading.Thread(target=self.heartbeat, daemon=True)
        self.thread.start()

    def tearDown(self):
        self.running = False
        self.thread.join(1.)
        self.status.unregister()
        self.cancel.unregister()
        self.state.unregister()
        self.intent_observer.unregister()

    def heartbeat(self):
        while self.running and not rospy.is_shutdown():
            message = GoalStatusArray(status_list=self.statuses)
            message.header.stamp = rospy.Time.now()
            self.status.publish(message)
            time.sleep(.05)

    def receive_cancel(self, message):
        self.cancellations.append(message)
        self.statuses = []  # The isolated action fixture acknowledges cancellation.

    def invoke(self, command, *extra):
        script = Path(__file__).resolve().parents[1] / 'scripts/control.py'
        return subprocess.run([sys.executable, str(script), command, '--timeout', '1.5', *extra],
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                              # Match a normal field shell, not this rostest's /ugv namespace.
                              env=dict(os.environ, ROS_NAMESPACE='/'), timeout=6.)

    def until(self, condition):
        deadline = time.monotonic() + 3.
        while time.monotonic() < deadline:
            if condition():
                return
            time.sleep(.02)
        self.fail('expected transport observation did not arrive')

    def test_real_handoff_busy_goal_cancel_and_latched_emergency(self):
        state = self.invoke('state')
        self.assertEqual(state.returncode, 0, state.stderr)
        self.assertTrue(json.loads(state.stdout)['manual_hold'])

        busy = self.invoke('release')
        self.assertNotEqual(busy.returncode, 0)
        self.assertIn('pending or active goal', busy.stderr)

        self.statuses = []
        time.sleep(.1)
        released = self.invoke('release')
        self.assertEqual(released.returncode, 0, released.stderr)
        state = json.loads(released.stdout)
        self.assertFalse(state['manual_hold'])
        self.assertEqual(state['takeover_generation'], 0)

        self.statuses = [GoalStatus(status=1)]
        held = self.invoke('hold')
        self.assertEqual(held.returncode, 0, held.stderr)
        state = json.loads(held.stdout)
        self.assertTrue(state['manual_hold'])
        self.assertEqual(state['takeover_generation'], 1)
        self.until(lambda: bool(self.cancellations))
        self.assertEqual(self.cancellations[-1].id, '')
        self.assertEqual(self.cancellations[-1].stamp, rospy.Time())

        self.statuses = []
        time.sleep(.1)
        released = self.invoke('release')
        self.assertEqual(released.returncode, 0, released.stderr)
        generation = json.loads(released.stdout)['takeover_generation']
        self.cancel.unregister()
        self.statuses = [GoalStatus(status=1)]
        no_cancel = self.invoke('hold')
        self.assertNotEqual(no_cancel.returncode, 0)
        self.assertIn('held confirmed; cancel unconfirmed', no_cancel.stderr)
        self.until(lambda: self.states and self.states[-1]['manual_hold']
                   and self.states[-1]['takeover_generation'] == generation + 1)

        conflict = rospy.Publisher('/cup_nav/manual_state', String, queue_size=1)
        time.sleep(.1)
        conflicted = self.invoke('state')
        self.assertNotEqual(conflicted.returncode, 0)
        self.assertIn('unique mux owner', conflicted.stderr)
        conflict.unregister()

        missing = self.invoke('state', '--state-topic', '/audit/missing', '--timeout', '.3')
        self.assertNotEqual(missing.returncode, 0)
        self.assertIn('no fresh state', missing.stderr)

        emergency = rospy.Publisher('/cup_nav/emergency_stop', Bool, queue_size=1)
        self.until(lambda: emergency.get_num_connections() > 0)
        emergency.publish(Bool(data=True))
        self.until(lambda: self.states and self.states[-1]['emergency_stop_latched'])
        emergency.publish(Bool(data=False))
        self.statuses = []
        stopped = self.invoke('release')
        self.assertNotEqual(stopped.returncode, 0)
        self.assertIn('emergency stop remains', stopped.stderr)
        self.assertTrue(self.states[-1]['manual_hold'])
        emergency.unregister()


if __name__ == '__main__':
    rospy.init_node('move_base', anonymous=False)
    rostest.rosrun('ugv_slicer_control', 'control_transport', ControlTransportTests)
