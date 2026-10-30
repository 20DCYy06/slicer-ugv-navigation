#!/usr/bin/env python3
"""Pure argument/pose tests; no ROS master or vehicle is required."""
import importlib.util
import math
from pathlib import Path
import unittest
from geometry_msgs.msg import TransformStamped


class GoalClientTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1] / 'scripts' / 'goal.py'
        spec = importlib.util.spec_from_file_location('goal_client', path)
        cls.client = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.client)

    def test_pose_preserves_coordinates_and_yaw(self):
        args = self.client.arguments(['send', '--x', '1.2', '--y', '-0.3', '--yaw', '1.57'])
        pose = self.client.goal_pose(args)
        self.assertEqual(pose.header.frame_id, 'odom')
        self.assertAlmostEqual(pose.pose.position.x, 1.2)
        self.assertAlmostEqual(pose.pose.position.y, -0.3)
        self.assertAlmostEqual(pose.pose.orientation.z, math.sin(1.57 / 2))
        self.assertAlmostEqual(pose.pose.orientation.w, math.cos(1.57 / 2))

    def test_alternate_action_and_frame(self):
        args = self.client.arguments(['--action', '/test/move_base', 'send', '--frame', 'local', '--x', '0', '--y', '0'])
        self.assertEqual(args.action, '/test/move_base')
        self.assertEqual(self.client.goal_pose(args).header.frame_id, 'local')

    def test_nonfinite_coordinates_rejected(self):
        for value in ('nan', 'inf', '-inf'):
            with self.subTest(value=value), self.assertRaises(SystemExit):
                self.client.arguments(['send', '--x=' + value, '--y', '0'])

    def test_invalid_timeout_rejected(self):
        for value in ('0', '-1', 'nan'):
            with self.subTest(value=value), self.assertRaises(SystemExit):
                self.client.arguments(['send', '--x', '0', '--y', '0', '--timeout', value])

    def test_empty_frame_rejected(self):
        with self.assertRaises(SystemExit):
            self.client.arguments(['send', '--x', '0', '--y', '0', '--frame', ''])

    def test_cancel_is_explicit(self):
        self.assertEqual(self.client.arguments(['cancel']).command, 'cancel')

    def test_relative_target_is_fixed_in_odom_once(self):
        args = self.client.arguments(['send', '--relative', '--x', '0.5', '--y', '0'])
        transform = TransformStamped()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = 12
        transform.transform.translation.y = -7
        transform.transform.rotation.z = math.sin(math.pi / 4)
        transform.transform.rotation.w = math.cos(math.pi / 4)
        pose = self.client.relative_goal_pose(args, transform)
        self.assertEqual(pose.header.frame_id, 'odom')
        self.assertAlmostEqual(pose.pose.position.x, 12)
        self.assertAlmostEqual(pose.pose.position.y, -6.5)
        self.assertAlmostEqual(pose.pose.orientation.z, math.sin(math.pi / 4))
        transform.transform.translation.x = 90
        self.assertAlmostEqual(pose.pose.position.x, 12)

    def test_plan_is_an_explicit_non_action_command(self):
        args = self.client.arguments(['plan', '--relative', '--x', '0.5', '--y', '0'])
        self.assertEqual(args.command, 'plan')
        self.assertTrue(args.relative)
        self.assertEqual(args.base_frame, 'base_link')

    def test_relative_goal_cannot_use_moving_target_frame(self):
        with self.assertRaises(SystemExit):
            self.client.arguments(['send', '--relative', '--frame', 'base_link', '--x', '0.5', '--y', '0'])


if __name__ == '__main__':
    unittest.main()
