#!/usr/bin/env python3
"""Pure geometry checks; no ROS master or ROS Python packages required."""
import math
import unittest

import preview_forward_goal as preview


class ForwardGoalGeometryTest(unittest.TestCase):
    def test_project_ten_metres_at_zero_45_and_90_degrees(self):
        for yaw, expected_x, expected_y in (
                (0.0, 11.0, -2.0),
                (math.pi / 4.0, 8.071067811865476, 5.071067811865475),
                (math.pi / 2.0, 1.0, 8.0)):
            with self.subTest(yaw=yaw):
                goal = preview.freeze_forward_goal(1.0, -2.0, yaw, 10.0)
                self.assertAlmostEqual(goal.x, expected_x)
                self.assertAlmostEqual(goal.y, expected_y)
                self.assertAlmostEqual(goal.yaw, yaw)

    def test_24_by_16_window_rejects_forward_goal_along_short_axis(self):
        east = preview.freeze_forward_goal(0.0, 0.0, 0.0, 10.0)
        north = preview.freeze_forward_goal(0.0, 0.0, math.pi / 2.0, 10.0)
        self.assertTrue(preview.check_window(east, -12.0, -8.0, 480, 320, 0.05, 0.35)['ok'])
        rejected = preview.check_window(north, -12.0, -8.0, 480, 320, 0.05, 0.35)
        self.assertFalse(rejected['ok'])
        self.assertIn('outside', rejected['reason'])

    def test_window_uses_actual_origin_and_requires_edge_margin(self):
        # Actual window is [3, 27) x [-5, 11), not centred at the robot.
        at_margin = preview.FrozenGoal(26.65, 10.65, 0.0)
        too_close = preview.FrozenGoal(26.66, 0.0, 0.0)
        self.assertTrue(preview.check_window(at_margin, 3.0, -5.0, 480, 320, 0.05, 0.35)['ok'])
        rejected = preview.check_window(too_close, 3.0, -5.0, 480, 320, 0.05, 0.35)
        self.assertFalse(rejected['ok'])
        self.assertIn('margin', rejected['reason'])
        on_outer_edge = preview.FrozenGoal(27.0, 0.0, 0.0)
        self.assertFalse(preview.check_window(on_outer_edge, 3.0, -5.0, 480, 320, 0.05, 0.0)['ok'])

    def test_frozen_goal_does_not_follow_a_later_odom_sample(self):
        odom = {'x': 2.0, 'y': 3.0, 'yaw': 0.0}
        frozen = preview.freeze_forward_goal(odom['x'], odom['y'], odom['yaw'], 10.0)
        odom.update(x=6.0, y=7.0, yaw=math.pi / 2.0)
        later = preview.freeze_forward_goal(odom['x'], odom['y'], odom['yaw'], 10.0)
        self.assertEqual(tuple(frozen), (12.0, 3.0, 0.0))
        self.assertAlmostEqual(later.x, 6.0)
        self.assertAlmostEqual(later.y, 17.0)
        with self.assertRaises(AttributeError):
            frozen.x = later.x

    def test_margin_comes_from_existing_footprint_circumscribed_radius(self):
        vertices = [[-0.165, -0.14], [-0.165, 0.14], [0.27, 0.14], [0.27, -0.14]]
        self.assertAlmostEqual(preview.footprint_radius(vertices), 0.304138126514911)


if __name__ == '__main__':
    unittest.main()
