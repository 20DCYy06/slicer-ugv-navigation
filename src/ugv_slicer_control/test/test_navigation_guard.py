#!/usr/bin/env python3
"""Pure Guard-policy regression: forbidden arcs stop instead of becoming spins."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from navigation_core import saturate_twist, slew_arc, planner_limits


class NavigationGuardTests(unittest.TestCase):
    SOURCE = dict(max_forward=.3, max_reverse=.02, max_angular=.6,
                  accel_linear=.5, accel_angular=2.)
    DWA = dict(max_vel_x=.125, min_vel_x=0., max_vel_trans=.125,
               max_vel_theta=.25, acc_lim_x=.2, acc_lim_trans=.2,
               acc_lim_theta=1.)

    def test_dwa_uses_its_caps_and_forbids_reverse_without_teb_parameters(self):
        limits = planner_limits(self.SOURCE, 'dwa_local_planner/DWAPlannerROS', self.DWA)
        self.assertEqual(limits, dict(max_forward=.125, max_reverse=0., max_angular=.25,
                                     accel_linear=.2, accel_angular=1.))
        self.assertEqual(saturate_twist(-.001, .25, limits['max_forward'],
                                       limits['max_reverse'], limits['max_angular']), (0., 0.))

    def test_dwa_also_respects_source_and_translational_caps(self):
        source = dict(self.SOURCE, max_forward=.1, accel_angular=.4)
        planner = dict(self.DWA, max_vel_trans=.08, acc_lim_trans=.15)
        self.assertEqual(planner_limits(source, 'dwa_local_planner/DWAPlannerROS', planner),
                         dict(max_forward=.08, max_reverse=0., max_angular=.25,
                              accel_linear=.15, accel_angular=.4))
        self.assertEqual(source['max_forward'], .1)

    def test_teb_reference_retains_source_and_selected_planner_bounds(self):
        teb = dict(max_vel_x=.2, max_vel_x_backwards=.01, max_vel_theta=.25,
                   acc_lim_x=.2, acc_lim_theta=1.)
        source = dict(self.SOURCE, max_forward=.1, max_reverse=0.)
        self.assertEqual(planner_limits(source, 'teb_local_planner/TebLocalPlannerROS', teb),
                         dict(max_forward=.1, max_reverse=0., max_angular=.25,
                              accel_linear=.2, accel_angular=1.))

    def test_unknown_plugin_is_rejected_instead_of_using_teb_limits(self):
        with self.assertRaisesRegex(ValueError, 'Unsupported local planner'):
            planner_limits(self.SOURCE, 'other/UnreviewedPlanner', self.DWA)

    def test_dwa_reverse_or_forward_floor_is_rejected(self):
        for minimum in (-.02, .01, math.nan, math.inf, False):
            with self.subTest(min_vel_x=minimum):
                with self.assertRaises(ValueError):
                    planner_limits(self.SOURCE, 'dwa_local_planner/DWAPlannerROS',
                                   dict(self.DWA, min_vel_x=minimum))

    def test_invalid_caps_are_rejected_before_any_output(self):
        for key in ('max_vel_x', 'max_vel_trans', 'max_vel_theta', 'acc_lim_x',
                    'acc_lim_trans', 'acc_lim_theta'):
            for invalid in (0., -.1, math.nan, math.inf, True):
                with self.subTest(key=key, value=invalid):
                    with self.assertRaises(ValueError):
                        planner_limits(self.SOURCE, 'dwa_local_planner/DWAPlannerROS',
                                       dict(self.DWA, **{key: invalid}))
        with self.assertRaises(ValueError):
            planner_limits(dict(self.SOURCE, max_reverse=-.02),
                           'dwa_local_planner/DWAPlannerROS', self.DWA)

    def test_forbidden_reverse_arc_rejects_the_whole_command(self):
        for velocity, omega in ((-.02, .25), (-.02, -.25), (-1.e-12, .05),
                                (-1.e-12, -.05), (-.02, 0.)):
            with self.subTest(vx=velocity, wz=omega):
                self.assertEqual(saturate_twist(velocity, omega, .125, 0., .25),
                                 (0., 0.))

    def test_original_pure_turn_remains_legal_in_both_directions(self):
        for velocity, omega in ((0., .25), (-0., -.25), (0., .02)):
            with self.subTest(vx=velocity, wz=omega):
                self.assertEqual(saturate_twist(velocity, omega, .125, 0., .25),
                                 (0., omega))

    def test_forward_saturation_preserves_arc_ratio(self):
        self.assertEqual(saturate_twist(.25, .25, .125, 0., .25), (.125, .125))
        self.assertEqual(saturate_twist(.1, -.5, .125, 0., .25), (.05, -.25))

    def test_positive_reverse_limit_preserves_existing_arc_ratio(self):
        self.assertEqual(saturate_twist(-.04, .2, .125, .02, .25), (-.02, .1))
        self.assertEqual(saturate_twist(-.01, -.5, .125, .02, .25), (-.005, -.25))

    def test_zero_and_rejected_command_stop_without_slew_residue(self):
        self.assertEqual(saturate_twist(0., 0., .125, 0., .25), (0., 0.))
        target = saturate_twist(-.02, .25, .125, 0., .25)
        self.assertEqual(slew_arc((.1, .25), target, .05, .2, 1.), (0., 0.))

    def test_non_finite_input_is_rejected(self):
        for velocity, omega in ((math.nan, .1), (.1, math.inf), (-math.inf, .1)):
            with self.subTest(vx=velocity, wz=omega):
                with self.assertRaises(ValueError):
                    saturate_twist(velocity, omega, .125, 0., .25)


if __name__ == '__main__':
    unittest.main()
