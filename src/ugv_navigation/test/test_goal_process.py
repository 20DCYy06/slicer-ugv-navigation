#!/usr/bin/env python3
"""TEST_ONLY real CLI process/Action checks against software_demo's plant."""
import signal
import os
import select
import subprocess
import time
import unittest

import actionlib
import rospy
import rostest
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionResult


class GoalProcessTest(unittest.TestCase):
    def test_cli_success_timeout_interrupt_and_explicit_cancel(self):
        results, commands = [], []
        result_sub = rospy.Subscriber('/ugv/move_base/result', MoveBaseActionResult,
                                      lambda msg: results.append(msg.status.status))
        raw_sub = rospy.Subscriber('/ugv/cmd_vel/nav_raw', Twist,
                                   lambda msg: commands.append((time.monotonic(), msg.linear.x, msg.angular.z)))
        client = actionlib.SimpleActionClient('/ugv/move_base', MoveBaseAction)
        self.assertTrue(client.wait_for_server(rospy.Duration(15)))
        base = ['rosrun', 'ugv_navigation', 'goal.py']
        prefixes = {}

        def start(*args):
            return subprocess.Popen(base + list(args), stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True)

        def finish(process, expected, timeout=10):
            try:
                output = prefixes.pop(process, '') + process.communicate(timeout=timeout)[0]
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate()
                self.fail('CLI process failed to terminate')
            print(output, flush=True)
            self.assertEqual(process.returncode, expected, output)
            return output

        def wait_for_goal_and_motion(process, since):
            deadline = time.monotonic() + 10
            output = ''
            while time.monotonic() < deadline:
                readable, _, _ = select.select([process.stdout], [], [], 0.1)
                if readable:
                    output += os.read(process.stdout.fileno(), 4096).decode()
                    prefixes[process] = output
                if 'Goal sent to ' in output and any(
                        stamp >= since and (abs(v) > .001 or abs(w) > .001)
                        for stamp, v, w in commands):
                    return
                if process.poll() is not None:
                    self.fail('CLI ended before sending/controlling its goal: ' + output)
            self.fail('CLI did not send a goal and produce motion in 10s: ' + output)

        try:
            output = finish(start('plan', '--relative', '--x', '0.5', '--y', '0'), 0)
            self.assertIn('no Action goal sent', output)
            self.assertEqual(results, [])
            self.assertFalse(any(abs(v) > 0.001 or abs(w) > 0.001 for _, v, w in commands))
            # CLI owns a real goal and receives its real SUCCEEDED result.
            finish(start('send', '--relative', '--x', '0.5', '--y', '0', '--timeout', '40'), 0, 45)
            finish_output = finish(start('send', '--x', '1.0', '--y', '0', '--timeout', '0.7'), 1)
            self.assertIn('terminal result received=True', finish_output)

            # Wait for this client to send before delivering a real SIGINT.
            since = time.monotonic()
            process = start('send', '--x', '1.0', '--y', '0')
            try:
                wait_for_goal_and_motion(process, since)
                self.assertIsNone(process.poll(), 'Interrupt fixture goal ended too early')
                process.send_signal(signal.SIGINT)
                self.assertIn('goal cancellation requested', finish(process, 130))
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()

            since = time.monotonic()
            process = start('send', '--x', '1.2', '--y', '0')
            try:
                wait_for_goal_and_motion(process, since)
                self.assertIsNone(process.poll(), 'Cancel fixture goal ended too early')
                finish(start('cancel'), 0)
                finish(process, 1)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate()

            deadline = time.monotonic() + 3
            while len(results) < 4 and time.monotonic() < deadline:
                time.sleep(0.02)
            self.assertEqual(results, [GoalStatus.SUCCEEDED] + [GoalStatus.PREEMPTED] * 3)
            since = time.monotonic()
            time.sleep(1)
            self.assertFalse(any(abs(v) > 0.001 or abs(w) > 0.001
                                 for stamp, v, w in commands if stamp >= since))
        finally:
            client.cancel_all_goals()
            result_sub.unregister()
            raw_sub.unregister()


if __name__ == '__main__':
    rospy.init_node('TEST_ONLY_goal_process')
    rostest.rosrun('ugv_navigation', 'goal_process', GoalProcessTest)
