#!/usr/bin/env python3
"""Pure offline fixtures; no ROS, device, master or recorded robot input."""
import math
import unittest

try:
    from analyze_motion_bag import analyze
except ModuleNotFoundError:
    analyze = None


def record(topic, at, message):
    return topic, message, at


def goal(identifier, at):
    return record('/ugv/move_base/goal', at, {
        'goal_id': {'id': identifier}, 'goal': {'target_pose': {
            'header': {'frame_id': 'odom'},
            'pose': {'position': {'x': .5, 'y': 0}, 'orientation': {'w': 1}}}}})


def result(identifier, at, status=3):
    return record('/ugv/move_base/result', at, {'status': {'goal_id': {'id': identifier}, 'status': status}})


def odom(at, x=0, y=0, yaw=0, v=0, w=0, topic='/odom'):
    return record(topic, at, {'header': {'frame_id': 'odom', 'stamp': at-100},
                            'pose': {'pose': {'position': {'x': x, 'y': y},
                                             'orientation': {'z': math.sin(yaw/2), 'w': math.cos(yaw/2)}}},
                            'twist': {'twist': {'linear': {'x': v}, 'angular': {'z': w}}}})


def velocity(at, v=0, w=0, topic='/cmd_vel'):
    return record(topic, at, {'linear': {'x': v}, 'angular': {'z': w}})


def diagnostic(at, event, instance='A', event_seq=1, **fields):
    import json
    fields.update(driver_instance_id=instance, event_seq=event_seq, host_monotonic_s=at+100)
    return record('/slicer_c30d_driver/diagnostics', at,
                  {'header': {'stamp': at-100}, 'status': [{'message': event, 'values': [
                      {'key': key, 'value': json.dumps(value)} for key, value in fields.items()]}]})


class MotionAnalysisTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(analyze, 'the offline motion analyzer must be implemented')

    def test_goals_match_exact_result_id_and_pose_deltas_use_initial_heading(self):
        records = [goal('one', 1), goal('two', 8), result('unrelated', 1.5),
                   result('two', 9, 4), result('one', 2),
                   odom(.9, 10, 20, math.pi/2), odom(2, 10, 20.5, math.pi/2),
                   odom(7, 10, 20.55, math.pi/2), odom(7.9, 10, 20.55, math.pi/2),
                   odom(9, 10, 20.6, math.pi/2), odom(14, 10, 20.6, math.pi/2)]
        data = analyze(records)
        first, second = data['goals']
        self.assertEqual(first['goal_id'], 'one')
        self.assertEqual(first['result']['receive_s'], 2)
        self.assertEqual(second['result']['status'], 4)
        poses = first['poses']['/odom']
        self.assertAlmostEqual(poses['terminal']['relative']['forward_m'], .5)
        self.assertAlmostEqual(poses['terminal']['relative']['left_m'], 0)
        self.assertAlmostEqual(poses['terminal_plus_5s']['relative']['forward_m'], .55)
        self.assertEqual(poses['before_goal']['header_stamp_s'], -99.1)
        self.assertEqual(data['time_basis'], 'bag_receive_s')

    def test_goal_yaw_is_normalized_and_odom_target_residual_wraps_across_pi(self):
        target = goal('one', 1)
        target_yaw = -math.pi+.03
        target[1]['goal']['target_pose']['pose']['orientation'] = {
            'z': 3*math.sin(target_yaw/2), 'w': 3*math.cos(target_yaw/2)}
        records = [target, result('one', 2), odom(.9),
                   odom(2, x=.4, y=-.1, yaw=math.pi-.02),
                   odom(7, x=.45, y=-.05, yaw=math.pi-.01),
                   odom(2, x=50, topic='/wheel/odom'),
                   odom(2, x=60, topic='/lio/odom')]
        motion = analyze(records)['goals'][0]
        self.assertAlmostEqual(motion['target'].get('yaw_rad', math.inf), target_yaw)
        terminal = motion['poses']['/odom']['terminal']['goal_frame_odom_residual']
        self.assertEqual(terminal['state'], 'observed')
        self.assertEqual(terminal['frame_id'], 'odom')
        self.assertAlmostEqual(terminal['x_m'], .1)
        self.assertAlmostEqual(terminal['y_m'], .1)
        self.assertAlmostEqual(terminal['position_norm_m'], math.sqrt(.02))
        self.assertAlmostEqual(terminal['yaw_rad'], .05)
        post = motion['poses']['/odom']['terminal_plus_5s']['goal_frame_odom_residual']
        self.assertAlmostEqual(post['yaw_rad'], .04)
        for topic in ('/wheel/odom', '/lio/odom', '/Odometry'):
            self.assertNotIn('goal_frame_odom_residual', motion['poses'][topic]['terminal'])

    def test_goal_odom_residual_is_unknown_for_frame_mismatch_or_invalid_quaternion(self):
        for invalid in ('frame', 'goal_quaternion', 'odom_quaternion'):
            with self.subTest(invalid=invalid):
                target, terminal = goal('one', 1), odom(2, x=.4)
                if invalid == 'frame':
                    target[1]['goal']['target_pose']['header']['frame_id'] = 'map'
                elif invalid == 'goal_quaternion':
                    target[1]['goal']['target_pose']['pose']['orientation'] = {'w': 0}
                else:
                    terminal[1]['pose']['pose']['orientation'] = {'w': 0}
                motion = analyze([target, result('one', 2), odom(.9), terminal])['goals'][0]
                residual = motion['poses']['/odom']['terminal'].get('goal_frame_odom_residual')
                self.assertIsNotNone(residual)
                self.assertEqual(residual['state'], 'unknown')
                if invalid == 'goal_quaternion':
                    self.assertEqual(motion['target']['state'], 'unknown')

    def test_goal_odom_post5_residual_stays_unknown_without_observation_coverage(self):
        motion = analyze([goal('one', 1), result('one', 2), odom(.9),
                          odom(2, x=.4), velocity(3)])['goals'][0]
        self.assertFalse(motion['requested_post_result_5s_covered'])
        post = motion['poses']['/odom']['terminal_plus_5s']
        self.assertEqual(post.get('goal_frame_odom_residual', {}).get('state'), 'unknown')
        self.assertNotIn('goal_frame_odom_residual', motion['poses']['/wheel/odom']['terminal_plus_5s'])

    def test_velocity_thresholds_and_continuous_zero_require_observed_coverage(self):
        records = [goal('one', 0), result('one', .5)]
        records += [velocity(0, .1), velocity(.1, 2e-6)]
        records += [velocity(i/10, 5e-7) for i in range(2, 56)]
        data = analyze(records, zero_epsilon=1e-6)
        stream = data['goals'][0]['velocity_chain']['/cmd_vel']
        self.assertEqual(stream['last_nonzero']['receive_s'], .1)
        self.assertEqual(stream['first_zero_after_last_nonzero']['receive_s'], .2)
        self.assertEqual(stream['sustained_zero']['state'], 'observed')
        self.assertEqual(data['thresholds']['zero_epsilon'], 1e-6)
        wheel = data['goals'][0]['wheel_stop']
        self.assertEqual(wheel['sustained_zero']['state'], 'unknown')

    def test_missing_or_stale_evidence_never_infers_timely_stop(self):
        data = analyze([goal('one', 1), result('one', 2), velocity(1.1, .1), velocity(2),
                        odom(.9), odom(2), odom(8)])
        motion = data['goals'][0]
        self.assertEqual(motion['velocity_chain']['/cmd_vel']['sustained_zero']['state'], 'unknown')
        self.assertEqual(motion['poses']['/odom']['terminal_plus_5s']['state'], 'unknown')
        no_result = analyze([goal('one', 1), velocity(1.1)])['goals'][0]
        self.assertIsNone(no_result['result'])
        self.assertEqual(no_result['velocity_chain']['/cmd_vel']['sustained_zero']['state'], 'unknown')

    def test_zero_acceptance_matches_skipped_sequences_and_16bit_wrap(self):
        records = [goal('one', 0), result('one', .5),
                   diagnostic(.01, 'serial_tx', event_seq=1, command_seq=65535, kind=17,
                              success=True, command_pwm_permille_abcd=[71]*4),
                   diagnostic(.1, 'command_rx', event_seq=2, valid=True, linear_x_m_s=0, angular_z_rad_s=0),
                   diagnostic(.11, 'serial_tx', event_seq=3, command_seq=0, kind=17,
                              success=True, command_pwm_permille_abcd=[0]*4),
                   diagnostic(.2, 'serial_tx', event_seq=4, command_seq=1, kind=17,
                              success=True, command_pwm_permille_abcd=[0]*4),
                   diagnostic(.3, 'telemetry', event_seq=5, accepted_command_seq=1,
                              mcu_tick_us=100, pwm_timer_ticks_abcd=[0]*4, raw_encoder_abcd=[0]*4,
                              measured_mm_s_abcd=[0]*4, voltage_v=12.3, flags=48),
                   diagnostic(.4, 'telemetry', event_seq=6, accepted_command_seq=1,
                              mcu_tick_us=200, pwm_timer_ticks_abcd=[0]*4, raw_encoder_abcd=[1, 0, 0, 0],
                              measured_mm_s_abcd=[10, 0, 0, 0], voltage_v=12.1, flags=48)]
        driver = analyze(records)['goals'][0]['driver_observations'][0]
        self.assertEqual(driver['first_zero_tx_after_last_nonzero']['command_seq'], 0)
        accepted = driver['accepted_zero_observation']
        self.assertEqual(accepted['accepted_command_seq'], 1)
        self.assertEqual(accepted['matched_zero_tx']['receive_s'], .2)
        self.assertEqual(driver['telemetry_pwm_zero']['receive_s'], .3)
        self.assertEqual(driver['encoder_after_pwm_zero']['changed_pairs'], 1)
        self.assertEqual(driver['voltage_v'], {'min': 12.1, 'max': 12.3})
        self.assertEqual(driver['max_abs_measured_mm_s'], 10)

    def test_restarts_and_event_sequence_gaps_are_separated(self):
        records = [goal('one', 0), result('one', 1),
                   diagnostic(.1, 'serial_tx', instance='A', event_seq=1, command_seq=7,
                              kind=17, success=True, command_pwm_permille_abcd=[0]*4),
                   diagnostic(.2, 'telemetry', instance='A', event_seq=3, accepted_command_seq=7),
                   diagnostic(.3, 'telemetry', instance='B', event_seq=1, accepted_command_seq=7)]
        data = analyze(records)
        a, b = data['driver_instances']
        self.assertEqual(a['event_seq_gap_ranges'], [[2, 2]])
        self.assertEqual(b['event_seq_gap_ranges'], [])
        self.assertEqual(b['accepted_zero_observation']['state'], 'unknown')
        self.assertEqual(a['first_zero_tx_after_last_nonzero']['state'], 'unknown')

    def test_failed_serial_write_is_not_an_accepted_zero_reference(self):
        records = [diagnostic(1, 'serial_tx', event_seq=1, command_seq=8, kind=17,
                              success=False, command_pwm_permille_abcd=[0]*4),
                   diagnostic(2, 'telemetry', event_seq=2, accepted_command_seq=8,
                              pwm_timer_ticks_abcd=[0]*4)]
        driver = analyze(records)['driver_instances'][0]
        self.assertEqual(driver['accepted_zero_observation']['state'], 'unknown')
        self.assertEqual(driver['telemetry_pwm_zero']['state'], 'unknown')

    def test_missing_sequence_cannot_supply_acceptance_evidence(self):
        missing_seq = [diagnostic(1, 'serial_tx', event_seq=1, kind=17,
                                  success=True, command_pwm_permille_abcd=[0]*4),
                       diagnostic(2, 'telemetry', event_seq=2, pwm_timer_ticks_abcd=[0]*4)]
        self.assertEqual(analyze(missing_seq)['driver_instances'][0]['accepted_zero_observation']['state'], 'unknown')

    def test_missing_mcu_tick_cannot_supply_encoder_evidence(self):
        missing_tick = [diagnostic(1, 'serial_tx', event_seq=1, kind=17, command_seq=7,
                                   success=True, command_pwm_permille_abcd=[0]*4),
                        diagnostic(2, 'telemetry', event_seq=2, accepted_command_seq=7,
                                   pwm_timer_ticks_abcd=[0]*4, raw_encoder_abcd=[0]*4),
                        diagnostic(3, 'telemetry', event_seq=3, accepted_command_seq=7, mcu_tick_us=20,
                                   pwm_timer_ticks_abcd=[0]*4, raw_encoder_abcd=[1]*4)]
        self.assertEqual(analyze(missing_tick)['driver_instances'][0]['encoder_after_pwm_zero']['state'], 'unknown')

    def test_wheel_stop_uses_explicit_linear_and_angular_thresholds(self):
        records = [goal('one', 0), result('one', .5), odom(0, v=.031, topic='/wheel/odom'),
                   odom(.1, w=.051, topic='/wheel/odom')]
        records += [odom(i/10, v=.03, w=.05, topic='/wheel/odom') for i in range(2, 56)]
        wheel = analyze(records)['goals'][0]['wheel_stop']
        self.assertEqual(wheel['last_nonzero']['receive_s'], .1)
        self.assertEqual(wheel['first_zero_after_last_nonzero']['receive_s'], .2)
        self.assertEqual(wheel['sustained_zero']['state'], 'observed')

    def test_a_later_goal_cannot_supply_the_previous_goals_zero_tail_or_post_pose(self):
        records = [goal('one', 0), result('one', 1), goal('two', 2), result('two', 3),
                   velocity(.1, .1), velocity(1.1), velocity(1.9), velocity(2.1, .1),
                   odom(0), odom(1, x=.5), odom(6, x=9)]
        records += [velocity(i/10) for i in range(31, 61)]
        first = analyze(records)['goals'][0]
        self.assertEqual(first['velocity_chain']['/cmd_vel']['last_nonzero']['receive_s'], .1)
        self.assertEqual(first['velocity_chain']['/cmd_vel']['sustained_zero']['state'], 'unknown')
        self.assertEqual(first['poses']['/odom']['terminal_plus_5s']['state'], 'unknown')

    def test_backward_mcu_frame_cannot_supply_pwm_zero_after_acceptance(self):
        records = [diagnostic(.1, 'serial_tx', event_seq=1, command_seq=6, kind=17,
                              success=True, command_pwm_permille_abcd=[71]*4),
                   diagnostic(1.9, 'telemetry', event_seq=2, accepted_command_seq=6,
                              mcu_tick_us=90, pwm_timer_ticks_abcd=[71]*4),
                   diagnostic(1, 'serial_tx', event_seq=3, command_seq=7, kind=17,
                              success=True, command_pwm_permille_abcd=[0]*4),
                   diagnostic(2, 'telemetry', event_seq=4, accepted_command_seq=7,
                              mcu_tick_us=200, pwm_timer_ticks_abcd=[71]*4),
                   diagnostic(3, 'telemetry', event_seq=5, accepted_command_seq=6,
                              mcu_tick_us=100, pwm_timer_ticks_abcd=[0]*4)]
        driver = analyze(records)['driver_instances'][0]
        self.assertEqual(driver['accepted_zero_observation']['state'], 'observed')
        self.assertEqual(driver['telemetry_pwm_zero']['state'], 'unknown')

    def test_invalid_last_encoder_pair_does_not_extend_observation_coverage(self):
        records = [diagnostic(0, 'serial_tx', event_seq=1, command_seq=8, kind=17,
                              success=True, command_pwm_permille_abcd=[0]*4)]
        for seq, (at, tick, encoder) in enumerate(((1, 100, 0), (1.1, 101, 0), (10, 100, 100)), 2):
            records.append(diagnostic(at, 'telemetry', event_seq=seq, accepted_command_seq=8,
                                      mcu_tick_us=tick, pwm_timer_ticks_abcd=[0]*4,
                                      raw_encoder_abcd=[encoder]*4))
        activity = analyze(records)['driver_instances'][0]['encoder_after_pwm_zero']
        self.assertEqual(activity['through_receive_s'], 1.1)

    def test_nonfinite_velocity_is_an_unknown_barrier_in_a_zero_tail(self):
        records = [goal('one', 0), result('one', .5), velocity(.1, .1), velocity(5.35, math.nan)]
        records += [velocity(i/10) for i in range(2, 56)]
        stream = analyze(records)['goals'][0]['velocity_chain']['/cmd_vel']
        self.assertEqual(stream['sustained_zero']['state'], 'unknown')
        self.assertEqual(stream['invalid_sample_count'], 1)

    def test_acceptance_freshness_needs_a_nearby_advancing_mcu_pair(self):
        for telemetry_times in ((1,), (1, 2)):
            with self.subTest(times=telemetry_times):
                records = [diagnostic(0, 'serial_tx', command_seq=8, kind=17, success=True,
                                      command_pwm_permille_abcd=[0]*4)]
                for i, at in enumerate(telemetry_times, 2):
                    records.append(diagnostic(at, 'telemetry', event_seq=i, accepted_command_seq=8,
                                              mcu_tick_us=i, pwm_timer_ticks_abcd=[0]*4))
                driver = analyze(records)['driver_instances'][0]
                self.assertEqual(driver['accepted_zero_observation']['state'], 'unknown')
                self.assertEqual(driver['raw_zero_acceptance_candidate']['accepted_command_seq'], 8)
                self.assertEqual(driver['telemetry_pwm_zero']['state'], 'unknown')

    def test_long_gap_does_not_extend_fresh_mcu_tick_coverage(self):
        records = [diagnostic(at, 'telemetry', event_seq=index, mcu_tick_us=index)
                   for index, at in enumerate((1, 1.1, 10), 1)]
        progress = analyze(records)['driver_instances'][0]['mcu_tick_progression']
        self.assertEqual(progress['through_receive_s'], 1.1)

    def test_speed_zero_acceptance_wraps_without_mixing_sent_units(self):
        records = [diagnostic(.01, 'serial_tx', event_seq=1, kind=17, command_seq=65534,
                              success=True, command_pwm_permille_abcd=[600]*4, speed_mm_s_abcd=[999]*4),
                   diagnostic(.05, 'serial_tx', event_seq=2, kind=16, command_seq=65535,
                              success=True, speed_mm_s_abcd=[-100, 100, -100, 100], command_pwm_permille_abcd=[700]*4),
                   diagnostic(.1, 'serial_tx', event_seq=3, kind=16, command_seq=0,
                              success=True, speed_mm_s_abcd=[0]*4),
                   diagnostic(.2, 'serial_tx', event_seq=4, kind=16, command_seq=1,
                              success=True, speed_mm_s_abcd=[0]*4)]
        records += [diagnostic(at, 'telemetry', event_seq=seq, accepted_command_seq=1,
                               mcu_tick_us=seq, pwm_timer_ticks_abcd=[0]*4,
                               raw_encoder_abcd=[seq]*4) for seq, at in ((5, .3), (6, .4))]
        driver = analyze(records)['driver_instances'][0]
        self.assertEqual(driver['max_abs_sent_pwm_permille'], 600)
        self.assertEqual(driver.get('max_abs_sent_wheel_speed_mm_s'), 100,
                         '0x10 speed references need their own mm/s maximum')
        self.assertEqual(driver['last_nonzero_pwm_tx']['kind'], 17)
        self.assertEqual(driver['last_nonzero_motion_tx']['kind'], 16)
        self.assertEqual(driver['last_nonzero_motion_tx']['command_unit'], 'mm/s')
        self.assertEqual(driver['first_zero_tx_after_last_nonzero']['command_seq'], 0)
        accepted = driver['accepted_zero_observation']
        self.assertEqual(accepted['accepted_command_seq'], 1)
        self.assertEqual(accepted['matched_zero_tx']['kind'], 16)
        self.assertEqual(accepted['matched_zero_tx']['speed_mm_s_abcd'], [0]*4)
        self.assertEqual(driver['telemetry_duty_reference_zero']['receive_s'], .3)
        self.assertEqual(driver['encoder_after_duty_reference_zero']['changed_pairs'], 1)

    def test_missing_wrong_kind_or_failed_speed_payload_is_not_a_zero_reference(self):
        bad_commands = [dict(kind=16, success=False, speed_mm_s_abcd=[0]*4),
                        dict(kind=16, success=True, command_pwm_permille_abcd=[0]*4),
                        dict(kind=17, success=True, speed_mm_s_abcd=[0]*4),
                        dict(kind=18, success=True, speed_mm_s_abcd=[0]*4)]
        for command in bad_commands:
            with self.subTest(command=command):
                records = [diagnostic(0, 'serial_tx', command_seq=7, **command)]
                records += [diagnostic(at, 'telemetry', event_seq=seq, accepted_command_seq=7,
                                       mcu_tick_us=seq, pwm_timer_ticks_abcd=[0]*4)
                            for seq, at in ((2, .1), (3, .2))]
                driver = analyze(records)['driver_instances'][0]
                self.assertEqual(driver.get('sent_motion_command_kinds'), [],
                                 'only kind-matched successful motion commands count')
                self.assertIsNone(driver['max_abs_sent_wheel_speed_mm_s'])
                self.assertIsNone(driver['max_abs_sent_pwm_permille'])
                self.assertEqual(driver['accepted_zero_observation']['state'], 'unknown')

    def test_speed_acceptance_does_not_cross_driver_instances(self):
        records = [diagnostic(0, 'serial_tx', kind=16, command_seq=7, success=True, speed_mm_s_abcd=[0]*4)]
        records += [diagnostic(at, 'telemetry', instance=instance, event_seq=seq,
                               accepted_command_seq=7, mcu_tick_us=seq, pwm_timer_ticks_abcd=[0]*4)
                    for instance, seq, at in (('A', 2, .1), ('A', 3, .2), ('B', 1, .3), ('B', 2, .4))]
        a, b = analyze(records)['driver_instances']
        self.assertEqual(a['accepted_zero_observation']['state'], 'observed')
        self.assertEqual(a['accepted_zero_observation']['matched_zero_tx']['kind'], 16)
        self.assertEqual(b['accepted_zero_observation']['state'], 'unknown')
        self.assertEqual(a['first_zero_tx_after_last_nonzero']['state'], 'unknown',
                         'accepted zero without earlier motion is not a movement-to-stop transition')

    def test_speed_zero_for_a_later_goal_cannot_finish_the_earlier_goal(self):
        records = [goal('one', 0), result('one', .5), goal('two', 1), result('two', 1.5),
                   diagnostic(.1, 'serial_tx', event_seq=1, kind=16, command_seq=7,
                              success=True, speed_mm_s_abcd=[100]*4),
                   diagnostic(1.1, 'serial_tx', event_seq=2, kind=16, command_seq=8,
                              success=True, speed_mm_s_abcd=[0]*4)]
        records += [diagnostic(at, 'telemetry', event_seq=seq, accepted_command_seq=8,
                               mcu_tick_us=seq, pwm_timer_ticks_abcd=[0]*4)
                    for seq, at in ((3, 1.2), (4, 1.3))]
        first, second = analyze(records)['goals']
        self.assertEqual(first['driver_observations'][0]['accepted_zero_observation']['state'], 'unknown')
        self.assertEqual(second['driver_observations'][0]['accepted_zero_observation']['state'], 'observed')

    def test_speed_acceptance_with_stale_telemetry_remains_unknown(self):
        records = [diagnostic(0, 'serial_tx', kind=16, command_seq=7,
                              success=True, speed_mm_s_abcd=[0]*4)]
        records += [diagnostic(at, 'telemetry', event_seq=seq, accepted_command_seq=7,
                               mcu_tick_us=seq, pwm_timer_ticks_abcd=[0]*4)
                    for seq, at in ((2, .1), (3, 1.1))]
        driver = analyze(records)['driver_instances'][0]
        self.assertEqual(driver['raw_zero_acceptance_candidate']['state'], 'observed')
        self.assertEqual(driver['raw_zero_acceptance_candidate']['matched_zero_tx']['kind'], 16)
        self.assertEqual(driver['accepted_zero_observation']['state'], 'unknown')
        self.assertEqual(driver['telemetry_pwm_zero']['state'], 'unknown')

    def test_body_rotation_and_double_zero_use_wire_units_and_independent_feedback(self):
        records = [diagnostic(0, 'serial_tx', event_seq=1, kind=16, command_seq=6,
                              success=True, speed_mm_s_abcd=[80]*4),
                   diagnostic(.1, 'serial_tx', event_seq=2, kind=22, command_seq=7,
                              success=True, body_vx_mm_s=0, body_wz_mrad_s=-250,
                              speed_mm_s_abcd=[0]*4, command_pwm_permille_abcd=[0]*4),
                   diagnostic(.2, 'command_tx', event_seq=3, command_seq=8, control_mode='body_speed',
                              target_body_vx_mm_s=0, target_body_wz_mrad_s=0,
                              linear_x_m_s=0., angular_z_rad_s=0.),
                   diagnostic(.21, 'serial_tx', event_seq=4, kind=22, command_seq=8,
                              success=True, body_vx_mm_s=0, body_wz_mrad_s=0)]
        records += [diagnostic(at, 'body_telemetry', event_seq=seq, mcu_us=seq*100,
                               ack_seq=8, version=1, who_am_i=0xea, imu_flags=7, imu_age_ms=10,
                               source=3, status_flags=15, vx_m_s=0., wz_rad_s=.02,
                               target_vx_m_s=0., target_wz_rad_s=0.)
                    for seq, at in ((5, .3), (6, .4))]
        driver = analyze(records)['driver_instances'][0]
        self.assertEqual(driver['sent_motion_command_kinds'], [16, 22])
        self.assertEqual(driver['last_nonzero_motion_tx']['kind'], 22,
                         'pure body rotation must not disappear behind four zero wheel values')
        self.assertEqual(driver['last_nonzero_body_speed_tx']['motion_mode'], 'body_speed')
        self.assertEqual(driver['last_nonzero_body_speed_tx']['command_unit'], 'mm/s and mrad/s')
        self.assertEqual(driver['max_abs_sent_body_vx_mm_s'], 0)
        self.assertEqual(driver['max_abs_sent_body_wz_mrad_s'], 250)
        self.assertEqual(driver['max_abs_sent_wheel_speed_mm_s'], 80)
        self.assertEqual(driver['first_zero_tx_after_last_nonzero']['command_seq'], 8)
        self.assertEqual(driver['body_command_targets'][0]['target_body_wz_mrad_s'], 0)
        self.assertEqual(driver['body_feedback']['advancing_mcu_frame_count'], 2)
        self.assertEqual(driver['body_feedback']['last_values']['wz_rad_s'], .02)
        accepted = driver['accepted_zero_observation']
        self.assertEqual(accepted['matched_zero_tx']['kind'], 22)
        self.assertEqual(accepted['ack_seq'], 8)
        self.assertEqual(driver['telemetry_pwm_zero']['state'], 'unknown')

    def test_body_linear_motion_is_nonzero_and_malformed_body_values_never_count_as_zero(self):
        for command in (dict(success=False, body_vx_mm_s=0, body_wz_mrad_s=0),
                        dict(success=True, body_vx_mm_s=0),
                        dict(success=True, speed_mm_s_abcd=[0]*4),
                        dict(success=True, body_vx_mm_s=math.nan, body_wz_mrad_s=0),
                        dict(success=True, body_vx_mm_s=0, body_wz_mrad_s=32768)):
            with self.subTest(command=command):
                records = [diagnostic(0, 'serial_tx', kind=22, command_seq=7,
                                      success=True, body_vx_mm_s=100, body_wz_mrad_s=0),
                           diagnostic(.1, 'serial_tx', kind=22, command_seq=8, **command)]
                driver = analyze(records)['driver_instances'][0]
                self.assertIsNotNone(driver['last_nonzero_motion_tx'], 'kind 22 linear motion must be observed')
                self.assertEqual(driver['last_nonzero_motion_tx']['body_vx_mm_s'], 100)
                self.assertEqual(driver['first_zero_tx_after_last_nonzero']['state'], 'unknown')

    def test_legacy_feedback_cannot_supply_body_feedback_freshness_or_body_acceptance(self):
        records = [diagnostic(0, 'serial_tx', kind=22, command_seq=8,
                              success=True, body_vx_mm_s=0, body_wz_mrad_s=0)]
        records += [diagnostic(at, 'telemetry', event_seq=seq, accepted_command_seq=8,
                               mcu_tick_us=seq, pwm_timer_ticks_abcd=[0]*4)
                    for seq, at in ((2, .1), (3, .2))]
        driver = analyze(records)['driver_instances'][0]
        self.assertEqual(driver['accepted_zero_observation']['state'], 'unknown')
        self.assertIn('body_feedback', driver)
        self.assertEqual(driver['body_feedback']['count'], 0)
        self.assertEqual(driver['body_feedback']['mcu_tick_progression']['state'], 'unknown')

    def test_repeated_body_ticks_and_bad_imu_are_reported_separately_from_legacy_ticks(self):
        records = [diagnostic(at, 'body_telemetry', event_seq=seq, mcu_us=100,
                              version=1, who_am_i=0xea, imu_flags=3, imu_age_ms=101,
                              source=0, status_flags=0x20f, vx_m_s=0., wz_rad_s=0.,
                              target_vx_m_s=0., target_wz_rad_s=0.)
                   for seq, at in ((1, .1), (2, .2))]
        records += [diagnostic(at, 'telemetry', event_seq=seq, mcu_tick_us=seq)
                    for seq, at in ((3, .3), (4, .4))]
        driver = analyze(records)['driver_instances'][0]
        self.assertIn('body_feedback', driver)
        body = driver['body_feedback']
        self.assertEqual(body['count'], 2)
        self.assertEqual(body['advancing_mcu_frame_count'], 1)
        self.assertEqual(body['rejected_or_missing_mcu_tick_count'], 1)
        self.assertEqual(body['invalid_imu_feedback_count'], 2)
        self.assertEqual(body['fault_feedback_count'], 2)
        self.assertEqual(body['mcu_tick_progression']['state'], 'unknown')

    def test_body_zero_ack_alone_is_not_a_motion_transition_or_fresh_feedback(self):
        records = [diagnostic(0, 'serial_tx', kind=22, command_seq=8,
                              success=True, body_vx_mm_s=0, body_wz_mrad_s=0),
                   diagnostic(.1, 'body_telemetry', ack_seq=8, mcu_us=100, version=1,
                              who_am_i=0xea, imu_flags=7, imu_age_ms=1, source=0, status_flags=15)]
        driver = analyze(records)['driver_instances'][0]
        self.assertEqual(driver['first_zero_tx_after_last_nonzero']['state'], 'unknown')
        self.assertEqual(driver['raw_zero_acceptance_candidate']['state'], 'observed')
        self.assertEqual(driver['accepted_zero_observation']['state'], 'unknown')

    def test_failed_imu_bit_is_invalid_even_with_all_ready_bits(self):
        records = [diagnostic(.1, 'body_telemetry', mcu_us=100, version=1,
                              who_am_i=0xea, imu_flags=15, imu_age_ms=0, source=0, status_flags=15)]
        body = analyze(records)['driver_instances'][0]['body_feedback']
        self.assertEqual(body['invalid_imu_feedback_count'], 1)

    def test_body_zero_confirmed_by_0x93_can_have_0x90_duty_and_encoder_observations(self):
        for ticks in ((100, 200), (100, 100)):
            with self.subTest(body_ticks=ticks):
                records = [diagnostic(.1, 'serial_tx', event_seq=1, kind=22, command_seq=7,
                                      success=True, body_vx_mm_s=0, body_wz_mrad_s=100),
                           diagnostic(.2, 'serial_tx', event_seq=2, kind=22, command_seq=8,
                                      success=True, body_vx_mm_s=0, body_wz_mrad_s=0)]
                records += [diagnostic(at, 'body_telemetry', event_seq=seq, mcu_us=tick,
                                       ack_seq=8, version=1, who_am_i=0xea, imu_flags=7,
                                       imu_age_ms=10, source=3, status_flags=15)
                            for seq, at, tick in ((3, .3, ticks[0]), (4, .4, ticks[1]))]
                records += [diagnostic(at, 'telemetry', event_seq=seq, mcu_tick_us=seq*100,
                                       accepted_command_seq=8, pwm_timer_ticks_abcd=[0]*4,
                                       raw_encoder_abcd=encoder)
                            for seq, at, encoder in ((5, .5, [42]*4), (6, .6, [43, 42, 42, 42]))]
                driver = analyze(records)['driver_instances'][0]
                if ticks[0] == ticks[1]:
                    self.assertEqual(driver['accepted_zero_observation']['state'], 'unknown')
                    self.assertEqual(driver['telemetry_duty_reference_zero']['state'], 'unknown')
                    self.assertEqual(driver['encoder_after_duty_reference_zero']['state'], 'unknown')
                else:
                    accepted = driver['accepted_zero_observation']
                    self.assertEqual(accepted['event'], 'body_telemetry')
                    self.assertEqual(accepted['matched_zero_tx']['command_seq'], 8)
                    duty = driver['telemetry_duty_reference_zero']
                    self.assertEqual(duty['state'], 'observed')
                    self.assertEqual(duty['receive_s'], .5)
                    self.assertEqual(duty['accepted_command_seq'], 8)
                    encoder = driver['encoder_after_duty_reference_zero']
                    self.assertEqual(encoder['changed_pairs'], 1)
                    self.assertEqual(encoder['sum_abs_delta_counts_abcd'], [1, 0, 0, 0])

    def test_body_duty_observation_requires_the_same_0x93_confirmed_zero_sequence(self):
        records = [diagnostic(.1, 'serial_tx', event_seq=1, kind=22, command_seq=8,
                              success=True, body_vx_mm_s=0, body_wz_mrad_s=0)]
        records += [diagnostic(at, 'body_telemetry', event_seq=seq, mcu_us=seq*100,
                               ack_seq=8, version=1, who_am_i=0xea, imu_flags=7,
                               imu_age_ms=10, source=3, status_flags=15)
                    for seq, at in ((2, .2), (3, .3))]
        records.append(diagnostic(.4, 'serial_tx', event_seq=4, kind=22, command_seq=9,
                                  success=True, body_vx_mm_s=0, body_wz_mrad_s=0))
        records += [diagnostic(at, 'telemetry', event_seq=seq, mcu_tick_us=seq*100,
                               accepted_command_seq=9, pwm_timer_ticks_abcd=[0]*4,
                               raw_encoder_abcd=[0]*4)
                    for seq, at in ((5, .5), (6, .6))]
        driver = analyze(records)['driver_instances'][0]
        self.assertEqual(driver['accepted_zero_observation']['matched_zero_tx']['command_seq'], 8)
        self.assertEqual(driver['telemetry_duty_reference_zero']['state'], 'unknown')

    def test_continuous_body_zeros_with_0x90_before_0x93_match_each_confirmed_sequence(self):
        for first_duty in (0, 71):
            with self.subTest(first_duty=first_duty):
                records = [diagnostic(0, 'serial_tx', event_seq=1, kind=22, command_seq=1,
                                      success=True, body_vx_mm_s=0, body_wz_mrad_s=100)]
                for sequence in range(2, 6):
                    sent_at = (sequence-1)*.05
                    records += [diagnostic(sent_at, 'serial_tx', event_seq=sequence*3-2,
                                           kind=22, command_seq=sequence, success=True,
                                           body_vx_mm_s=0, body_wz_mrad_s=0),
                                diagnostic(sent_at+.03, 'telemetry', event_seq=sequence*3-1,
                                           mcu_tick_us=sequence*50000, accepted_command_seq=sequence,
                                           pwm_timer_ticks_abcd=[first_duty if sequence == 2 else 0]*4,
                                           raw_encoder_abcd=[sequence]*4),
                                diagnostic(sent_at+.031, 'body_telemetry', event_seq=sequence*3,
                                           mcu_us=sequence*50000, ack_seq=sequence, version=1,
                                           who_am_i=0xea, imu_flags=7, imu_age_ms=1,
                                           source=3, status_flags=15)]
                driver = analyze(records, max_age_s=.1)['driver_instances'][0]
                self.assertEqual(driver['accepted_zero_observation']['matched_zero_tx']['command_seq'], 2)
                duty = driver['telemetry_duty_reference_zero']
                self.assertEqual(duty['state'], 'observed')
                expected_sequence = 2 if first_duty == 0 else 3
                self.assertEqual(duty['accepted_command_seq'], expected_sequence)
                self.assertAlmostEqual(duty['receive_s'], .08 if first_duty == 0 else .13)
                self.assertAlmostEqual(duty['body_zero_confirmation_receive_s'], .081 if first_duty == 0 else .131)
                self.assertAlmostEqual(duty['body_zero_freshness_confirmed_through_receive_s'], .131)
                encoder = driver['encoder_after_duty_reference_zero']
                self.assertEqual(encoder['changed_pairs'], 3 if first_duty == 0 else 2)

    def test_old_body_confirmation_cannot_be_kept_fresh_by_later_0x90(self):
        records = [diagnostic(0, 'serial_tx', kind=22, command_seq=8,
                              success=True, body_vx_mm_s=0, body_wz_mrad_s=0)]
        records += [diagnostic(at, 'body_telemetry', event_seq=seq, mcu_us=seq*100,
                               ack_seq=8, version=1, who_am_i=0xea, imu_flags=7,
                               imu_age_ms=1, source=3, status_flags=15)
                    for seq, at in ((2, .1), (3, .2))]
        records += [diagnostic(at, 'telemetry', event_seq=seq, mcu_tick_us=seq*100,
                               accepted_command_seq=8, pwm_timer_ticks_abcd=[0]*4,
                               raw_encoder_abcd=[0]*4)
                    for seq, at in ((4, 1.), (5, 1.1))]
        driver = analyze(records)['driver_instances'][0]
        self.assertEqual(driver['accepted_zero_observation']['state'], 'observed')
        self.assertEqual(driver['telemetry_duty_reference_zero']['state'], 'unknown')


if __name__ == '__main__':
    unittest.main()
