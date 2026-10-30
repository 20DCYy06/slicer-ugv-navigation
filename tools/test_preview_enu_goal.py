#!/usr/bin/env python3
"""Hand-derived offline ENU fixtures; no ROS, network or vehicle is used."""
import json
import math
from pathlib import Path
import subprocess
import sys
import unittest


TOOL = Path(__file__).with_name('preview_enu_goal.py')
# Run the real CLI with site packages disabled. Any attempted network call,
# external process or ROS import makes the child fail before that operation.
ISOLATED_RUNNER = r'''
import runpy
import sys
def reject_external(event, args):
    if (event.startswith('socket.') or event.startswith('subprocess.')
            or event in ('os.system', 'os.exec', 'os.posix_spawn')):
        raise RuntimeError('offline preview attempted an external operation: ' + event)
    if event == 'import' and args[0].split('.')[0] in (
            'rospy', 'roslib', 'actionlib', 'tf2_ros', 'geometry_msgs', 'nav_msgs'):
        raise RuntimeError('offline preview attempted a ROS import: ' + args[0])
sys.addaudithook(reject_external)
sys.argv = sys.argv[1:]
runpy.run_path(sys.argv[0], run_name='__main__')
'''


class PreviewEnuGoalTest(unittest.TestCase):
    def cli(self, *args):
        return subprocess.run(
            [sys.executable, '-I', '-S', '-c', ISOLATED_RUNNER, str(TOOL),
             *map(str, args)], text=True, capture_output=True, timeout=5)

    def report(self, *args):
        result = self.cli(*args)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stderr, '')
        return json.loads(result.stdout)

    def test_identity_preserves_the_declared_target_and_units(self):
        report = self.report('--east', 2, '--north', -3, '--target-yaw', .25,
                             '--map-from-enu', 0, 0, 0)
        self.assertTrue(report['preview_only'])
        self.assertEqual(report['units'], {'position': 'm', 'yaw': 'rad'})
        self.assertEqual(report['input'],
                         {'frame_id': 'enu', 'east': 2., 'north': -3., 'target_yaw': .25})
        self.assertEqual(report['map_target']['frame_id'], 'map')
        self.assertAlmostEqual(report['map_target']['x'], 2.)
        self.assertAlmostEqual(report['map_target']['y'], -3.)
        self.assertAlmostEqual(report['map_target']['yaw'], .25)

    def test_ninety_degree_rotation_then_translation_uses_correct_direction(self):
        report = self.report('--east', 2, '--north', 3, '--target-yaw', math.pi / 6,
                             '--map-from-enu', 10, -2, math.pi / 2)
        self.assertAlmostEqual(report['map_target']['x'], 7.)
        self.assertAlmostEqual(report['map_target']['y'], 0.)
        self.assertAlmostEqual(report['map_target']['yaw'], 2.0943951023931953)
        self.assertEqual(report['transforms'][0]['direction'], 'map_from_enu')

    def test_composition_applies_enu_to_odom_before_odom_to_map(self):
        report = self.report('--east', 2, '--north', 3, '--target-yaw', math.pi / 6,
                             '--odom-from-enu', 10, -2, math.pi / 2,
                             '--map-from-odom', -1, 4, -math.pi / 2)
        # Hand calculation: ENU (2,3) -> odom (7,0) -> map (-1,-3).
        self.assertAlmostEqual(report['map_target']['x'], -1.)
        self.assertAlmostEqual(report['map_target']['y'], -3.)
        self.assertAlmostEqual(report['map_target']['yaw'], .5235987755982988)
        self.assertEqual([t['direction'] for t in report['transforms']],
                         ['odom_from_enu', 'map_from_odom'])

    def test_new_map_transform_snapshot_changes_composed_result(self):
        report = self.report('--east', 2, '--north', 3,
                             '--odom-from-enu', 10, -2, math.pi / 2,
                             '--map-from-odom', 4, 5, 0)
        self.assertAlmostEqual(report['map_target']['x'], 11.)
        self.assertAlmostEqual(report['map_target']['y'], 5.)

    def test_unspecified_target_yaw_is_not_invented(self):
        for transforms in (
                ['--map-from-enu', 10, -2, math.pi / 2],
                ['--odom-from-enu', 10, -2, math.pi / 2,
                 '--map-from-odom', -1, 4, -math.pi / 2]):
            with self.subTest(transforms=transforms):
                report = self.report('--east', 2, '--north', 3, *transforms)
                self.assertNotIn('target_yaw', report['input'])
                self.assertNotIn('yaw', report['map_target'])

    def test_negative_rotation_and_wrapped_yaw(self):
        report = self.report('--east', 2, '--north', 3,
                             '--target-yaw', -3 * math.pi / 4,
                             '--map-from-enu', 1, 2, -math.pi / 2)
        self.assertAlmostEqual(report['map_target']['x'], 4.)
        self.assertAlmostEqual(report['map_target']['y'], 0.)
        self.assertAlmostEqual(report['map_target']['yaw'], 2.356194490192345)

    def test_angles_outside_one_turn_are_normalized(self):
        report = self.report('--east', 2, '--north', 3,
                             '--target-yaw', 4 * math.pi + math.pi / 6,
                             '--map-from-enu', 10, -2, 5 * math.pi / 2)
        self.assertAlmostEqual(report['map_target']['x'], 7.)
        self.assertAlmostEqual(report['map_target']['y'], 0.)
        self.assertAlmostEqual(report['map_target']['yaw'], 2.0943951023931953)

    def test_nonfinite_coordinates_yaw_and_transform_values_are_rejected(self):
        for value in ('nan', 'inf', '-inf'):
            cases = [
                ['--east=' + value, '--north', 0, '--map-from-enu', 0, 0, 0],
                ['--east', 0, '--north=' + value, '--map-from-enu', 0, 0, 0],
                ['--east', 0, '--north', 0, '--target-yaw=' + value,
                 '--map-from-enu', 0, 0, 0],
            ]
            for component in range(3):
                transform = [0, 0, 0]
                transform[component] = value
                cases.extend([
                    ['--east', 0, '--north', 0, '--map-from-enu', *transform],
                    ['--east', 0, '--north', 0, '--odom-from-enu', *transform,
                     '--map-from-odom', 0, 0, 0],
                    ['--east', 0, '--north', 0, '--odom-from-enu', 0, 0, 0,
                     '--map-from-odom', *transform],
                ])
            for args in cases:
                with self.subTest(args=args):
                    result = self.cli(*args)
                    self.assertEqual(result.returncode, 2, result.stderr)
                    self.assertEqual(result.stdout, '')

    def test_transform_choice_is_required_mutually_exclusive_and_complete(self):
        invalid = [[], ['--odom-from-enu', 0, 0, 0],
                   ['--map-from-odom', 0, 0, 0],
                   ['--map-from-enu', 0, 0, 0, '--odom-from-enu', 0, 0, 0,
                    '--map-from-odom', 0, 0, 0],
                   ['--map-from-enu', 0, 0, 0, '--map-from-odom', 0, 0, 0]]
        for transforms in invalid:
            with self.subTest(transforms=transforms):
                result = self.cli('--east', 1, '--north', 2, *transforms)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertEqual(result.stdout, '')

    def test_origin_and_measurement_notes_are_retained_without_verification_claim(self):
        report = self.report('--east', 0, '--north', 0, '--map-from-enu', 0, 0, 0,
                             '--origin-label', 'synthetic-fixture',
                             '--alignment-source', 'hand calculation, no RTK measurement')
        registration = report['registration']
        self.assertEqual(registration['origin_label'], 'synthetic-fixture')
        self.assertEqual(registration['source'], 'hand calculation, no RTK measurement')
        self.assertFalse(registration['verified'])

    def test_cli_runs_without_ros_network_or_an_executable_goal_command(self):
        report = self.report('--east', 1, '--north', 2, '--map-from-enu', 0, 0, 0)
        self.assertTrue(report['preview_only'])
        self.assertAlmostEqual(report['map_target']['x'], 1.)
        self.assertNotIn('send_command', report)
        self.assertNotIn('command', report)

    def test_finite_inputs_that_overflow_do_not_produce_invalid_json(self):
        result = self.cli('--east', '1e308', '--north', 0,
                          '--map-from-enu', '1e308', 0, 0)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertEqual(result.stdout, '')


if __name__ == '__main__':
    unittest.main()
