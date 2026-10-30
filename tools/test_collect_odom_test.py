#!/usr/bin/env python3
"""Pure checks for the passive collector, also run in a device-free container."""
import unittest

try:
    import collect_odom_test as capture
except ImportError:
    capture = None


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(capture, 'collector must exist')

    def test_default_topic_selection_has_feedback_without_large_sensor_streams(self):
        topics = capture.topic_map('ugv')
        self.assertEqual(topics['lio'], '/lio/odom')
        self.assertEqual(topics['wheel'], '/wheel/odom')
        self.assertEqual(topics['driver_diagnostics'], '/slicer_c30d_driver/diagnostics')
        self.assertEqual(topics['action_status'], '/ugv/move_base/status')
        self.assertFalse(any(x in ' '.join(topics.values()) for x in ('/lidar', 'image', '/costmap')))
        self.assertEqual(capture.topic_map('/ugv_check/')['action_status'], '/ugv_check/move_base/status')

    def test_navigation_capture_selects_costmaps_both_local_planners_and_speed_chain(self):
        try:
            topics = capture.topic_map('/ugv_check/', navigation=True)
        except TypeError:
            self.fail('navigation capture must be selectable without unbounded topic recording')
        for topic in ('/ugv_check/move_base/global_costmap/costmap',
                      '/ugv_check/move_base/global_costmap/costmap_updates',
                      '/ugv_check/move_base/global_costmap/footprint',
                      '/ugv_check/move_base/local_costmap/costmap',
                      '/ugv_check/move_base/local_costmap/costmap_updates',
                      '/ugv_check/move_base/local_costmap/footprint',
                      '/ugv_check/move_base/GlobalPlanner/plan',
                      '/ugv_check/move_base/TebLocalPlannerROS/global_plan',
                      '/ugv_check/move_base/TebLocalPlannerROS/local_plan',
                      '/ugv_check/move_base/DWAPlannerROS/global_plan',
                      '/ugv_check/move_base/DWAPlannerROS/local_plan',
                      '/ugv_check/move_base/current_goal',
                      '/ugv_check/move_base_simple/goal',
                      '/slicer_nav_arc_guard/adjustment', '/cmd_vel/nav_raw',
                      '/cmd_vel/nav', '/cmd_vel', '/cmd_vel/mission', '/teleop/cmd_vel',
                      '/slicer_c30d_driver/diagnostics'):
            self.assertIn(topic, topics.values())
        self.assertNotIn('/livox/lidar', topics.values())
        self.assertNotIn('/cloud_registered_body', topics.values())
        self.assertNotIn('/cup_nav/manual_intent', topics.values())

    def test_perception_capture_adds_raw_and_intermediate_clouds_only_when_selected(self):
        try:
            topics = capture.topic_map('ugv', perception=True)
        except TypeError:
            self.fail('short perception capture must explicitly select the sensor chain')
        for topic in ('/livox/lidar', '/livox/imu', '/cloud_registered',
                      '/cloud_registered_body', '/cloud_nav_filtered',
                      '/scan_raw', '/scan', '/scan_front', '/ugv/perception_test/scan'):
            self.assertIn(topic, topics.values())
        self.assertNotIn('/Laser_map', topics.values(), 'do not record the accumulated map')
        self.assertNotIn('/livox/lidar', capture.topic_map('ugv').values())

    def test_perception_snapshot_reads_only_selected_sensor_and_fastlio_parameters(self):
        class Master:
            def getSystemState(self, caller):
                return 1, '', [[], [], []]

            def getTopicTypes(self, caller):
                return 1, '', []

            def getParam(self, caller, path):
                return 1, '', {'threshold': .08, 'api_key': 'do not export'}

        try:
            parameters = capture.snapshot(Master(), 'ugv', perception=True)['parameters']
        except TypeError:
            self.fail('perception snapshots must include effective sensor parameters')
        selected = ('/slicer_cloud_to_scan', '/slicer_arm_cloud_filter', '/slicer_scan_filter',
                    '/slicer_geometry', '/ugv/perception_test/cloud_to_scan',
                    '/preprocess', '/publish', '/filter_size_surf',
                    '/filter_size_map', '/point_filter_num', '/frame_id', '/user_config_path',
                    '/common', '/mapping')
        for path in selected:
            self.assertIn(path, parameters)
            self.assertEqual(parameters[path], {'threshold': .08, 'api_key': '[redacted]'})
        self.assertNotIn('/', parameters, 'never read the whole ROS parameter root')
        default = capture.snapshot(Master(), 'ugv')['parameters']
        self.assertNotIn('/slicer_cloud_to_scan', default)
        self.assertNotIn('/mapping', default)

    def test_receive_frequency_is_independent_of_header_clock(self):
        stream = capture.StreamStats()
        stream.observe(10., 100., 99.)
        stream.observe(10.1, 100.1, 99.1)
        stream.observe(10.2, 100.2, 99.2)
        result = stream.summary(10.3)
        self.assertAlmostEqual(result['receive_hz'], 10.)
        self.assertAlmostEqual(result['last_receive_age_s'], .1)
        self.assertAlmostEqual(result['last_header_age_s'], 1.)

    def test_repeated_and_old_headers_do_not_look_like_fresh_lio(self):
        stream = capture.StreamStats()
        for i, stamp in enumerate((8., 8., 7.)):
            stream.observe(10.+i*.1, 10.+i*.1, stamp)
        result = stream.summary(10.3)
        self.assertEqual(result['repeated_header_count'], 1)
        self.assertEqual(result['backward_header_count'], 1)
        self.assertGreater(result['last_header_age_s'], 3.)
        self.assertGreater(result['receive_hz'], 9.)

    def test_empty_stream_has_no_manufactured_rate_or_age(self):
        result = capture.StreamStats().summary(100.)
        self.assertEqual(result['count'], 0)
        self.assertIsNone(result['receive_hz'])
        self.assertIsNone(result['last_receive_age_s'])

    def test_selected_parameter_credentials_are_not_copied(self):
        result = capture.redact({'odom0': '/lio/odom', 'password': 'do not copy',
                                 'nested': {'api_key': 'secret', 'frequency': 30}})
        self.assertEqual(result['odom0'], '/lio/odom')
        self.assertEqual(result['password'], '[redacted]')
        self.assertEqual(result['nested']['api_key'], '[redacted]')
        self.assertEqual(result['nested']['frequency'], 30)

    def test_record_command_is_only_a_bounded_topic_whitelist(self):
        command = capture.record_command('/tmp/capture', ['/odom', '/tf'], 5)
        self.assertEqual(command[:2], ['rosbag', 'record'])
        self.assertIn('/tmp/capture/motion.bag', command)
        self.assertEqual(command[-2:], ['/odom', '/tf'])
        self.assertNotIn('-a', command)
        self.assertNotIn('pub', command)

    def body_parameters(self):
        return dict(control_mode='body_speed', provenance=dict(control_mode='body_speed',
                    mcu_configuration=dict(request_nonce='4294967295', ack_nonce='4294967295',
                                           accepted=1, configured=True),
                    body_extension=dict(request_nonce='42', ack_nonce='42', version=1,
                                        who_am_i=0xea, imu_flags=7, source=0)))

    def mode_summary(self, params):
        self.assertTrue(hasattr(capture, 'driver_mode_summary'), 'body runtime provenance must be summarized')
        return capture.driver_mode_summary(params)

    def body_summary(self, rows, now, stale_after):
        self.assertTrue(hasattr(capture, 'body_feedback_summary'), 'body feedback must be summarized')
        return capture.body_feedback_summary(rows, now, stale_after)

    def test_explicit_body_provenance_checks_configuration_and_extension_without_claiming_motion(self):
        summary = self.mode_summary(self.body_parameters())
        self.assertEqual(summary['driver_reported_control_mode'], 'body_speed')
        self.assertEqual(summary['driver_requested_control_mode'], 'body_speed')
        self.assertEqual(summary['findings'], [])
        self.assertEqual(summary['driver_body_extension']['source'], 0,
                         'STOP source is allowed during capability/zero-speed observations')
        self.assertIn('physical', summary['mode_note'])

    def test_body_requested_parameter_or_zero_frames_cannot_replace_runtime_provenance(self):
        summary = self.mode_summary(dict(control_mode='body_speed'))
        self.assertEqual(summary['driver_reported_control_mode'], 'unknown')
        self.assertIn('body_speed runtime provenance not confirmed', summary['findings'])

    def test_body_nonce_capability_and_imu_failures_remain_findings(self):
        cases = [('mcu_configuration', 'ack_nonce', '41', 'body_speed MCU configuration not confirmed'),
                 ('mcu_configuration', 'accepted', 0, 'body_speed MCU configuration not confirmed'),
                 ('mcu_configuration', 'configured', False, 'body_speed MCU configuration not confirmed'),
                 ('body_extension', 'ack_nonce', '41', 'body_speed extension capability not confirmed'),
                 ('body_extension', 'version', 2, 'body_speed extension capability not confirmed'),
                 ('body_extension', 'who_am_i', 0, 'body_speed extension capability not confirmed'),
                 ('body_extension', 'imu_flags', 3, 'body_speed extension capability not confirmed'),
                 ('body_extension', 'imu_flags', 15, 'body_speed extension capability not confirmed')]
        for section, key, value, finding in cases:
            with self.subTest(section=section, key=key):
                params = self.body_parameters()
                params['provenance'][section][key] = value
                self.assertIn(finding, self.mode_summary(params)['findings'])

    def test_legacy_wheel_speed_provenance_keeps_existing_behavior(self):
        summary = self.mode_summary(dict(control_mode='wheel_speed',
                                         provenance=dict(control_mode='wheel_speed')))
        self.assertEqual(summary['findings'], [])
        self.assertEqual(summary['driver_reported_control_mode'], 'wheel_speed')

    def test_body_feedback_reports_independent_ticks_imu_health_and_faults(self):
        base = dict(version=1, who_am_i=0xea, imu_flags=7, imu_age_ms=10, source=3,
                    vx_m_s=.1, wz_rad_s=-.2, target_vx_m_s=.1, target_wz_rad_s=-.2,
                    ack_seq=8, status_flags=15)
        rows = [(1., dict(base, mcu_us=100)), (1.1, dict(base, mcu_us=200)),
                (1.2, dict(base, mcu_us=200, imu_age_ms=101)),
                (1.3, dict(base, mcu_us=150, imu_flags=3, status_flags=0x10f))]
        summary = self.body_summary(rows, 1.4, .5)
        self.assertEqual(summary['count'], 4)
        self.assertEqual(summary['advancing_mcu_frame_count'], 2)
        self.assertEqual(summary['repeated_or_backward_mcu_tick_count'], 2)
        self.assertEqual(summary['invalid_imu_feedback_count'], 2)
        self.assertEqual(summary['fault_feedback_count'], 1)
        self.assertEqual(summary['last_values']['wz_rad_s'], -.2)
        self.assertIn('body feedback: repeated/backward MCU timestamps', summary['findings'])
        self.assertIn('body feedback: invalid/stale IMU observations', summary['findings'])
        self.assertIn('body feedback: MCU fault flags observed', summary['findings'])

    def test_missing_or_receive_stale_body_feedback_remains_a_finding(self):
        empty = self.body_summary([], 100., .5)
        self.assertIn('body feedback: no messages', empty['findings'])
        row = dict(version=1, mcu_us=100, who_am_i=0xea, imu_flags=7, imu_age_ms=0,
                   source=0, status_flags=15)
        stale = self.body_summary([(1., row)], 2., .5)
        self.assertIn('body feedback: receive stale', stale['findings'])

    def test_failed_imu_bit_overrides_all_ready_bits_in_body_feedback(self):
        row = dict(version=1, mcu_us=100, who_am_i=0xea, imu_flags=15, imu_age_ms=0,
                   source=0, status_flags=15)
        summary = self.body_summary([(1., row)], 1.1, .5)
        self.assertEqual(summary['invalid_imu_feedback_count'], 1)
        self.assertIn('body feedback: invalid/stale IMU observations', summary['findings'])


if __name__ == '__main__':
    unittest.main()
