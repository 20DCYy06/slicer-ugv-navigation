#!/usr/bin/env python3
"""Synthetic offline CLI fixtures; no measured origin or registration is used."""
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


TOOL = Path(__file__).with_name('preview_gnss_goal.py')
ISOLATED_RUNNER = r'''
import runpy
import sys
def reject_external(event, args):
    if (event.startswith('socket.') or event.startswith('subprocess.')
            or event in ('os.system', 'os.exec', 'os.posix_spawn')):
        raise RuntimeError('GNSS preview attempted an external operation: ' + event)
    if event == 'import' and args[0].split('.')[0] in (
            'rospy', 'roslib', 'actionlib', 'tf2_ros', 'geometry_msgs', 'nav_msgs'):
        raise RuntimeError('GNSS preview attempted a ROS import: ' + args[0])
    if event == 'open' and isinstance(args[0], str):
        if args[0].startswith(('/home/rescue/industrial_cup_ros1/', '/dev/')):
            raise RuntimeError('GNSS preview accessed an external source or device')
        if isinstance(args[1], str) and any(c in args[1] for c in 'wax+'):
            raise RuntimeError('GNSS preview attempted a file write')
sys.addaudithook(reject_external)
sys.argv = sys.argv[1:]
runpy.run_path(sys.argv[0], run_name='__main__')
'''


class PreviewGnssGoalTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='synthetic_gnss_preview_')
        self.addCleanup(self.temporary.cleanup)
        self.origin_file = Path(self.temporary.name) / 'origin.json'
        self.write_origin({'datum': 'WGS84', 'origin_id': 'SYNTHETIC_TEST_ONLY',
                           'origin': {'latitude': 0., 'longitude': 0.,
                                      'altitude_ellipsoid_m': 0.}})

    def write_origin(self, record):
        self.origin_file.write_text(json.dumps(record), encoding='utf-8')

    def cli(self, *args, default_origin=False):
        supplied = [] if default_origin else ['--origin-file', str(self.origin_file)]
        return subprocess.run(
            [sys.executable, '-I', '-S', '-B', '-c', ISOLATED_RUNNER, str(TOOL),
             *supplied, *map(str, args)], text=True, capture_output=True, timeout=5,
            cwd=self.temporary.name)

    def report(self, *args):
        result = self.cli(*args)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stderr, '')
        return json.loads(result.stdout)

    def rejected(self, *args, default_origin=False):
        result = self.cli(*args, default_origin=default_origin)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertEqual(result.stdout, '')
        self.assertNotIn('Traceback', result.stderr)
        return result.stderr

    def test_same_point_has_zero_enu_and_keeps_origin_identity(self):
        report = self.report('--latitude', 0, '--longitude', 0, '--altitude-ellipsoid', 0,
                             '--map-from-enu', 0, 0, 0)
        self.assertTrue(report['preview_only'])
        self.assertEqual(report['input'], {'frame_id': 'enu', 'east': 0., 'north': 0., 'up': 0.})
        self.assertEqual(report['map_target'], {'frame_id': 'map', 'x': 0., 'y': 0.})
        self.assertEqual(report['enu_origin']['origin_id'], 'SYNTHETIC_TEST_ONLY')
        self.assertEqual(report['enu_origin']['datum'], 'WGS84')
        self.assertEqual(report['gnss_input'], {'datum': 'WGS84', 'latitude': 0.,
                                             'longitude': 0., 'altitude_ellipsoid_m': 0.})
        self.assertEqual(report['physical_point'], 'gnss_antenna')
        self.assertFalse(report['lever_arm_compensation_applied'])
        self.assertFalse(report['registration']['verified'])
        self.assertNotIn('send_command', report)
        self.assertNotIn('command', report)

    def test_east_offset_rotates_and_translates_in_the_declared_direction(self):
        # At the equator, 8.983152841195214e-6 degrees of longitude is one metre East.
        report = self.report('--latitude', 0, '--longitude', 8.983152841195214e-6,
                             '--altitude-ellipsoid', 0,
                             '--map-from-enu', 10, -2, math.pi / 2)
        self.assertAlmostEqual(report['input']['east'], 1., places=7)
        self.assertAlmostEqual(report['input']['north'], 0., places=7)
        self.assertAlmostEqual(report['map_target']['x'], 10., places=7)
        self.assertAlmostEqual(report['map_target']['y'], -1., places=7)
        self.assertNotIn('yaw', report['map_target'])

    def test_height_is_ellipsoid_metres_and_up_is_reported_without_a_map_z(self):
        report = self.report('--latitude', 0, '--longitude', 0, '--altitude-ellipsoid', 3,
                             '--map-from-enu', 0, 0, 0)
        self.assertEqual(report['input']['up'], 3.)
        self.assertEqual(report['map_target'], {'frame_id': 'map', 'x': 0., 'y': 0.})

    def test_latitude_offset_is_north_in_metres(self):
        # Equatorial meridian radius is 6335439.3272928195 metres.
        report = self.report('--latitude', 9.04369477050382e-6, '--longitude', 0,
                             '--altitude-ellipsoid', 0, '--map-from-enu', 0, 0, 0)
        self.assertAlmostEqual(report['input']['east'], 0., places=7)
        self.assertAlmostEqual(report['input']['north'], 1., places=7)

    def test_small_negative_enu_values_are_not_reparsed_as_cli_options(self):
        report = self.report('--latitude=-0.000000000001', '--longitude=-0.000000000001',
                             '--altitude-ellipsoid', 0, '--map-from-enu', 0, 0, 0)
        self.assertAlmostEqual(report['map_target']['x'], -0.00000011131949079327357, places=14)
        self.assertAlmostEqual(report['map_target']['y'], -0.00000011057427582159436, places=14)

    def test_small_negative_transform_and_yaw_keep_their_numeric_values(self):
        report = self.report('--latitude', 0, '--longitude', 0, '--altitude-ellipsoid', 0,
                             '--map-from-enu', '-0.0000000001', 0, 0,
                             '--target-yaw=-0.0000000001')
        self.assertEqual(report['map_target']['x'], -1e-10)
        self.assertEqual(report['map_target']['yaw'], -1e-10)

    def test_composed_transforms_apply_enu_to_odom_before_odom_to_map(self):
        report = self.report('--latitude', 0, '--longitude', 8.983152841195214e-6,
                             '--altitude-ellipsoid', 0, '--target-yaw', 0,
                             '--odom-from-enu', 10, -2, math.pi / 2,
                             '--map-from-odom', -1, 4, -math.pi / 2)
        # ENU (1,0) -> odom (10,-1) -> map (-2,-6).
        self.assertAlmostEqual(report['map_target']['x'], -2., places=7)
        self.assertAlmostEqual(report['map_target']['y'], -6., places=7)
        self.assertAlmostEqual(report['map_target']['yaw'], 0., places=7)
        self.assertEqual([t['direction'] for t in report['transforms']],
                         ['odom_from_enu', 'map_from_odom'])

    def test_explicit_zero_yaw_is_rotated_but_an_omitted_yaw_stays_absent(self):
        base = ['--latitude', 0, '--longitude', 0, '--altitude-ellipsoid', 0,
                '--map-from-enu', 0, 0, math.pi / 2]
        explicit = self.report(*base, '--target-yaw', 0)
        self.assertEqual(explicit['input']['target_yaw'], 0.)
        self.assertAlmostEqual(explicit['map_target']['yaw'], math.pi / 2)
        omitted = self.report(*base)
        self.assertNotIn('target_yaw', omitted['input'])
        self.assertNotIn('yaw', omitted['map_target'])

    def test_unconfigured_default_origin_is_rejected_without_selecting_the_input(self):
        error = self.rejected('--latitude', 0, '--longitude', 0, '--altitude-ellipsoid', 0,
                              '--map-from-enu', 0, 0, 0, default_origin=True)
        self.assertIn('datum', error)

    def test_missing_bad_or_non_wgs84_origin_is_rejected_without_writing_it(self):
        cases = [None, [], {}, {'datum': None, 'origin': {}},
                 {'datum': 'CGCS2000', 'origin': {'latitude': 0, 'longitude': 0,
                                                'altitude_ellipsoid_m': 0}},
                 {'datum': 'WGS84', 'origin': None}, {'datum': 'WGS84', 'origin': {}},
                 {'datum': 'WGS84', 'origin': {'latitude': 0, 'longitude': 0}},
                 {'datum': 'WGS84', 'origin_id': [], 'origin': {
                     'latitude': 0, 'longitude': 0, 'altitude_ellipsoid_m': 0}}]
        for field, value in [('latitude', None), ('longitude', 181),
                             ('latitude', True), ('longitude', '0'),
                             ('altitude_ellipsoid_m', float('nan')),
                             ('altitude_ellipsoid_m', float('inf'))]:
            origin = {'latitude': 0, 'longitude': 0, 'altitude_ellipsoid_m': 0}
            origin[field] = value
            cases.append({'datum': 'WGS84', 'origin': origin})
        for record in cases:
            with self.subTest(record=record):
                self.write_origin(record)
                before = self.origin_file.read_bytes()
                self.rejected('--latitude', 0, '--longitude', 0, '--altitude-ellipsoid', 0,
                              '--map-from-enu', 0, 0, 0)
                self.assertEqual(self.origin_file.read_bytes(), before)
        self.origin_file.write_text('{invalid JSON', encoding='utf-8')
        self.rejected('--latitude', 0, '--longitude', 0, '--altitude-ellipsoid', 0,
                      '--map-from-enu', 0, 0, 0)
        self.origin_file.unlink()
        self.rejected('--latitude', 0, '--longitude', 0, '--altitude-ellipsoid', 0,
                      '--map-from-enu', 0, 0, 0)

    def test_origin_id_is_optional_and_origin_coordinates_define_the_reference(self):
        self.write_origin({'datum': 'WGS84', 'origin': {
            'latitude': 31, 'longitude': 118, 'altitude_ellipsoid_m': 20}})
        report = self.report('--latitude', 31, '--longitude', 118, '--altitude-ellipsoid', 20,
                             '--map-from-enu', 4, 5, 0)
        self.assertEqual(report['map_target'], {'frame_id': 'map', 'x': 4., 'y': 5.})
        self.assertEqual(report['enu_origin']['origin']['latitude'], 31.)

    def test_invalid_or_incomplete_target_coordinates_are_rejected(self):
        for field, value in [('latitude', 'nan'), ('longitude', 'inf'),
                             ('altitude-ellipsoid', '-inf'), ('latitude', 91),
                             ('longitude', -181)]:
            values = {'latitude': 0, 'longitude': 0, 'altitude-ellipsoid': 0}
            values[field] = value
            options = [option for key, number in values.items()
                       for option in ['--' + key + '=' + str(number)]]
            with self.subTest(field=field, value=value):
                self.rejected(*options, '--map-from-enu', 0, 0, 0)
        self.rejected('--latitude', 0, '--longitude', 0, '--map-from-enu', 0, 0, 0)

    def test_invalid_transform_choice_and_nonfinite_yaw_are_rejected(self):
        base = ['--latitude', 0, '--longitude', 0, '--altitude-ellipsoid', 0]
        for options in [[], ['--odom-from-enu', 0, 0, 0],
                        ['--map-from-enu', 0, 0, 0, '--map-from-odom', 0, 0, 0],
                        ['--map-from-enu', 0, 0, 0, '--odom-from-enu', 0, 0, 0,
                         '--map-from-odom', 0, 0, 0],
                        ['--map-from-enu', 0, 0, 'nan'],
                        ['--map-from-enu', 0, 0, 0, '--target-yaw', 'nan']]:
            with self.subTest(options=options):
                self.rejected(*base, *options)


class SiteConfiguredPreviewTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='synthetic_gnss_site_')
        self.addCleanup(self.temporary.cleanup)
        self.site = Path(self.temporary.name) / 'site.yaml'
        self.record = {
            'datum': 'WGS84',
            'origin': {'latitude': 0., 'longitude': 0., 'altitude_ellipsoid_m': 0.},
            'map_from_enu': [10., -2., math.pi / 2],
            'antenna_in_base_link_m': [0.1, -0.02, 0.25],
        }

    def cli(self, *options, default_site=False, content=None):
        # JSON is also YAML; the CLI must load this through the real PyYAML
        # reader, rather than a mocked or second hand-written parser.
        self.site.write_text(content if content is not None else json.dumps(self.record))
        supplied = [] if default_site else ['--site-config', str(self.site)]
        return subprocess.run(
            [sys.executable, '-I', '-B', '-c', ISOLATED_RUNNER, str(TOOL),
             '--latitude', '0', '--longitude', '0.000008983152841195214',
             '--altitude-ellipsoid', '0', *supplied, *map(str, options)],
            text=True, capture_output=True, timeout=5, cwd=self.temporary.name)

    def report(self, *options, **kwargs):
        result = self.cli(*options, **kwargs)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stderr, '')
        return json.loads(result.stdout)

    def rejected(self, *options, **kwargs):
        result = self.cli(*options, **kwargs)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertEqual(result.stdout, '')
        self.assertNotIn('Traceback', result.stderr)
        return result.stderr

    def test_site_supplies_common_origin_and_map_transform_without_extra_cli_fields(self):
        report = self.report()
        self.assertAlmostEqual(report['map_target']['x'], 10., places=7)
        self.assertAlmostEqual(report['map_target']['y'], -1., places=7)
        self.assertNotIn('yaw', report['map_target'])
        self.assertEqual(report['antenna_in_base_link_m'], [0.1, -0.02, 0.25])
        self.assertEqual(report['physical_point'], 'gnss_antenna')
        self.assertFalse(report['lever_arm_compensation_applied'])
        self.assertFalse(report['registration']['verified'])

    def test_site_accepts_comments_and_unknown_antenna_without_faking_zero_offset(self):
        report = self.report(content="""# Synthetic fixture, not a measured site.
datum: WGS84
origin: {latitude: 0, longitude: 0, altitude_ellipsoid_m: 0}
map_from_enu: [0, 0, 0]  # tx metres, ty metres, yaw radians
antenna_in_base_link_m: [null, null, null]
""")
        self.assertAlmostEqual(report['map_target']['x'], 1., places=7)
        self.assertEqual(report['antenna_in_base_link_m'], [None, None, None])
        self.assertFalse(report['lever_arm_compensation_applied'])

    def test_explicit_cli_transform_overrides_site_transform_and_preserves_optional_yaw(self):
        omitted = self.report('--map-from-enu', 3, 4, 0)
        explicit = self.report('--map-from-enu', 3, 4, 0, '--target-yaw', 0)
        self.assertAlmostEqual(omitted['map_target']['x'], 4., places=7)
        self.assertEqual(omitted['map_target']['y'], 4.)
        self.assertNotIn('yaw', omitted['map_target'])
        self.assertEqual(explicit['map_target']['yaw'], 0.)

    def test_unconfigured_default_site_is_rejected_without_adopting_target_as_origin(self):
        self.assertIn('datum', self.rejected(default_site=True))

    def test_site_and_legacy_origin_file_are_not_silently_mixed(self):
        self.rejected('--origin-file', str(self.site))

    def test_invalid_datum_transform_and_antenna_values_are_rejected(self):
        cases = [
            ('datum', 'CGCS2000'), ('datum', None),
            ('map_from_enu', None), ('map_from_enu', [0, 0]),
            ('map_from_enu', [0, None, 0]), ('map_from_enu', [0, True, 0]),
            ('map_from_enu', [0, 0, float('inf')]),
            ('antenna_in_base_link_m', [0, 0]),
            ('antenna_in_base_link_m', [0, '1', 0]),
            ('antenna_in_base_link_m', [0, float('nan'), 0]),
        ]
        for key, value in cases:
            with self.subTest(key=key, value=value):
                before = dict(self.record)
                self.record[key] = value
                self.rejected()
                self.record = before
        self.rejected(content='origin: [broken YAML')


if __name__ == '__main__':
    unittest.main()
