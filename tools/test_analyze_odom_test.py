#!/usr/bin/env python3
"""Offline fixtures only. No robot logs, network, serial port or ROS master."""
import math
import unittest

try:
    from analyze_odom_test import analyze
except ImportError:
    analyze = None


def odom(topic, at, x=0., y=0., yaw=0., header=None, frame='odom', child='base_link', v=0.):
    return topic, {'header': {'stamp': at if header is None else header, 'frame_id': frame},
                   'child_frame_id': child,
                   'pose': {'pose': {'position': {'x': x, 'y': y},
                                     'orientation': {'x': 0., 'y': 0., 'z': math.sin(yaw/2), 'w': math.cos(yaw/2)}}},
                   'twist': {'twist': {'linear': {'x': v}, 'angular': {'z': 0.}}}}, at


def cmd(at, v):
    return '/cmd_vel', {'linear': {'x': v}, 'angular': {'z': 0.}}, at


class OdomAnalysisTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(analyze, 'offline analyzer must exist')

    def paired(self):
        rows=[]
        for i in range(6):
            at=10+i*.1; d=i*.1
            rows += [odom('/odom', at, 100, 200+d, math.pi/2, v=1),
                     odom('/lio/odom', at, -20, 40+d, math.pi/2, v=1),
                     odom('/wheel/odom', at, d, 0., 0., v=1)]
        return rows

    def test_independent_origins_compare_start_frame_increments(self):
        report=analyze(self.paired())
        pose=report['odometry']['/odom']['displacement']
        self.assertAlmostEqual(pose['forward_m'], .5)
        self.assertAlmostEqual(pose['left_m'], 0.)
        pair=report['increment_comparisons']['/lio/odom -> /odom']
        self.assertEqual(pair['state'], 'estimated')
        self.assertAlmostEqual(pair['forward_difference_m'], 0.)
        self.assertEqual(pair['physical_localization_error']['state'], 'unknown')
        self.assertNotIn('absolute_position_error_m', pair)

    def test_different_sample_rates_share_interpolated_endpoints(self):
        rows=[odom('/lio/odom',10+i*.2,100+i*.2) for i in range(4)]
        rows += [odom('/odom',10.05+i*.1,-50+.05+i*.1) for i in range(6)]
        pair=analyze(rows)['increment_comparisons']['/lio/odom -> /odom']
        self.assertEqual(pair['state'],'estimated')
        self.assertAlmostEqual(pair['source_increment']['displacement_m'],.5)
        self.assertAlmostEqual(pair['source_increment']['path_length_m'],.5)
        self.assertAlmostEqual(pair['forward_difference_m'],0.)
        self.assertEqual(pair['endpoint_receive_skew_s'],0.)

    def test_header_anomalies_are_separate_from_record_frequency(self):
        rows=[odom('/odom', 10+i*.1, i*.01, header=h) for i,h in enumerate([10.,10.,9.,11.,10.4])]
        timing=analyze(rows)['topics']['/odom']
        self.assertAlmostEqual(timing['bag_record_rate_hz'], 10.)
        self.assertEqual(timing['header']['duplicate_count'], 1)
        self.assertEqual(timing['header']['backward_count'], 2)
        self.assertEqual(timing['header']['future_vs_record_count'], 1)
        self.assertGreater(timing['header']['max_record_minus_header_s'], 1.)

    def test_stale_or_duplicate_lio_cannot_make_comparison_pass(self):
        rows=[]
        for i in range(10):
            at=10+i*.1
            rows += [odom('/odom',at,i*.1), odom('/lio/odom',at,i*.1,header=min(at,10.2))]
        report=analyze(rows,max_gap=.2)
        self.assertEqual(report['increment_comparisons']['/lio/odom -> /odom']['state'],'unknown')
        self.assertGreater(report['topics']['/lio/odom']['header']['duplicate_count'],0)

    def test_no_overlap_or_missing_topic_is_unknown(self):
        rows=[odom('/odom',10),odom('/odom',10.1,.1),odom('/lio/odom',12),odom('/lio/odom',12.1,.1)]
        self.assertEqual(analyze(rows)['increment_comparisons']['/lio/odom -> /odom']['state'],'unknown')
        self.assertEqual(analyze([])['odometry']['/odom']['displacement']['state'],'unknown')

    def test_frame_change_and_suspected_reset_do_not_become_error(self):
        rows=self.paired()+[odom('/odom',10.6,100,200.6,math.pi/2,frame='new_odom')]
        report=analyze(rows)
        self.assertEqual(report['odometry']['/odom']['displacement']['state'],'unknown')
        rows=[odom('/odom',10),odom('/odom',10.1,.01),odom('/odom',10.2,10)]
        report=analyze(rows)
        self.assertEqual(report['odometry']['/odom']['suspected_pose_jump_count'],1)
        self.assertEqual(report['odometry']['/odom']['displacement']['state'],'unknown')

    def test_yaw_wrap_uses_short_signed_rotation(self):
        rows=[odom('/odom',10,yaw=math.radians(179)),odom('/odom',10.1,yaw=math.radians(-179))]
        self.assertAlmostEqual(analyze(rows)['odometry']['/odom']['displacement']['yaw_rad'],math.radians(2))

    def test_manual_values_are_labelled_and_not_fabricated(self):
        report=analyze(self.paired())
        self.assertEqual(report['manual_measurements']['real_displacement_m']['state'],'unknown')
        report=analyze(self.paired(),measurements={'real_displacement_m':.48,'real_stop_distance_m':.03})
        self.assertEqual(report['manual_measurements']['real_displacement_m']['source'],'manual_input')
        self.assertAlmostEqual(report['manual_comparisons']['/odom']['displacement_minus_manual_m'],.02)
        self.assertEqual(report['stopping']['physical_stop_confirmed']['state'],'unknown')
        self.assertEqual(report['manual_measurements']['real_stop_distance_m']['value'],.03)

    def test_bad_measurements_and_windows_rejected(self):
        for data in [{'real_displacement_m':float('nan')},{'real_stop_distance_m':-.1},
                     {'window':{'start_s':12,'end_s':10}},{'stop':{'command_zero_s':12,'settled_s':11}}]:
            with self.subTest(data=data),self.assertRaises(ValueError):analyze(self.paired(),measurements=data)

    def test_after_zero_is_pose_estimate_not_physical_stop(self):
        rows=[cmd(10,.1),cmd(10.2,0),cmd(10.3,0),cmd(10.4,0)]
        rows += [odom('/odom',10,0),odom('/odom',10.1,.01),odom('/odom',10.2,.02),
                 odom('/odom',10.3,.06),odom('/odom',10.4,.07)]
        report=analyze(rows)
        self.assertEqual(report['stopping']['command_zero']['bag_record_s'],10.2)
        self.assertAlmostEqual(report['stopping']['estimated_after_zero']['/odom']['displacement_m'],.05)
        self.assertEqual(report['stopping']['physical_stop_confirmed']['state'],'unknown')

    def test_manual_stop_window_requires_fresh_coverage(self):
        data={'stop':{'command_zero_s':10.2,'settled_s':11.}}
        rows=[odom('/odom',10),odom('/odom',10.2,.1),odom('/odom',12,.9)]
        report=analyze(rows,measurements=data)
        self.assertEqual(report['stopping']['estimated_after_zero']['/odom']['state'],'unknown')

    def test_host_monotonic_not_subtracted_from_ros_time(self):
        rows=self.paired()
        rows += [('/slicer_c30d_driver/diagnostics',{'header':{'stamp':10.1},'status':[{'message':'telemetry','values':[
            {'key':'host_monotonic_s','value':'5000.0'},{'key':'mcu_tick_us','value':'4294967200'},
            {'key':'pwm_timer_ticks_abcd','value':'[0,0,0,0]'}]}]},10.1),
            ('/slicer_c30d_driver/diagnostics',{'header':{'stamp':10.2},'status':[{'message':'telemetry','values':[
            {'key':'host_monotonic_s','value':'5000.1'},{'key':'mcu_tick_us','value':'100'},
            {'key':'pwm_timer_ticks_abcd','value':'[0,0,0,0]'}]}]},10.2)]
        report=analyze(rows)
        self.assertEqual(report['time_basis'],'bag_record_s')
        self.assertNotIn('physical_latency_s',report['mcu'])
        self.assertEqual(report['mcu']['telemetry_events'],2)

    def test_null_template_uses_existing_coverage(self):
        data={'real_displacement_m':None,'real_forward_m':None,'real_lateral_m':None,
              'real_stop_distance_m':None,'window':{'start_s':None,'end_s':None},
              'stop':{'command_zero_s':None,'settled_s':None}}
        report=analyze(self.paired(),measurements=data)
        self.assertEqual(report['odometry']['/odom']['displacement']['state'],'estimated')
        self.assertEqual(report['increment_comparisons']['/lio/odom -> /odom']['state'],'estimated')

    def test_input_record_order_anomaly_is_visible(self):
        rows=[odom('/odom',10),odom('/odom',10.2,.2),odom('/odom',10.1,.1)]
        self.assertEqual(analyze(rows)['topics']['/odom']['record_nonadvancing_count'],1)

    def test_pose_speed_is_separate_from_reported_zero_twist(self):
        rows=[odom('/odom',10,0,v=0),odom('/odom',10.1,.1,v=0)]
        report=analyze(rows)['odometry']['/odom']
        self.assertAlmostEqual(report['pose_derived_speed']['by_header_time_m_s']['mean'],1.)
        self.assertEqual(report['reported_velocity']['linear_x_m_s']['max'],0.)

    def test_tf_edges_and_static_stamps_reported_separately(self):
        rows=[('/tf',{'transforms':[{'header':{'frame_id':'odom','stamp':10},'child_frame_id':'base_link'}]},10),
              ('/tf_static',{'transforms':[{'header':{'frame_id':'base_link','stamp':0},'child_frame_id':'laser_link'}]},10)]
        report=analyze(rows)
        self.assertIn('odom -> base_link',report['tf']['dynamic'])
        self.assertIn('base_link -> laser_link',report['tf']['static'])
        self.assertEqual(report['tf']['static']['base_link -> laser_link']['header']['zero_count'],1)


class RosbagCliIntegration(unittest.TestCase):
    def test_real_bag_read_only_cli_with_remapping_and_manual_values(self):
        try:
            import rosbag
            import rospy
            from nav_msgs.msg import Odometry
            from geometry_msgs.msg import Twist, TransformStamped
            from tf2_msgs.msg import TFMessage
            from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        except ImportError:
            self.skipTest('Noetic/rosbag integration runs in the device-free offline container')
        import json
        import pathlib
        import subprocess
        import sys
        import tempfile
        with tempfile.TemporaryDirectory(prefix='TEST_ONLY_odom_analysis_') as name:
            root=pathlib.Path(name)
            (root/'manifest.json').write_text(json.dumps({'topic_map':{'odom':'/test/fused','lio':'/test/lio'}}))
            (root/'measurements.json').write_text(json.dumps({'real_displacement_m':.48,'real_stop_distance_m':.02,
                'window':{'start_s':None,'end_s':None},'stop':{'command_zero_s':None,'settled_s':None}}))
            with rosbag.Bag(str(root/'motion.bag'),'w') as bag:
                for i in range(6):
                    at=10+i*.1
                    for topic,x,y,header in [('/test/fused',100.,200+i*.1,at),('/test/lio',-20.,40+i*.1,at),('/Odometry',0.,i*.1,at+100)]:
                        msg=Odometry();msg.header.stamp=rospy.Time.from_sec(header);msg.header.frame_id='odom';msg.child_frame_id='base_link'
                        msg.pose.pose.position.x=x;msg.pose.pose.position.y=y;msg.pose.pose.orientation.z=math.sin(math.pi/4);msg.pose.pose.orientation.w=math.cos(math.pi/4)
                        bag.write(topic,msg,rospy.Time.from_sec(at))
                    msg=Twist();msg.linear.x=.1 if i<2 else 0.;bag.write('/cmd_vel',msg,rospy.Time.from_sec(at))
                    tf=TransformStamped();tf.header.stamp=rospy.Time.from_sec(at);tf.header.frame_id='odom';tf.child_frame_id='base_link';tf.transform.rotation.w=1.
                    bag.write('/tf',TFMessage([tf]),rospy.Time.from_sec(at))
                    diag=DiagnosticArray();diag.header.stamp=rospy.Time.from_sec(at);status=DiagnosticStatus();status.name='TEST_ONLY';status.message='telemetry'
                    status.values=[KeyValue(k,json.dumps(v)) for k,v in {'driver_instance_id':'TEST_ONLY','host_monotonic_s':5000+i*.1,
                        'mcu_tick_us':i*100000,'measured_mm_s_abcd':[100]*4,'pwm_timer_ticks_abcd':[170]*4,'voltage_v':12.}.items()]
                    diag.status=[status];bag.write('/slicer_c30d_driver/diagnostics',diag,rospy.Time.from_sec(at))
            cmd=[sys.executable,str(pathlib.Path(__file__).with_name('analyze_odom_test.py')),str(root),'--measurements',str(root/'measurements.json')]
            result=subprocess.run(cmd,capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            report=json.loads((root/'report.json').read_text())
            self.assertEqual(report['increment_comparisons']['/lio/odom -> /odom']['state'],'estimated')
            self.assertAlmostEqual(report['odometry']['/odom']['displacement']['forward_m'],.5)
            self.assertEqual(report['odometry']['/Odometry']['displacement']['state'],'unknown')
            self.assertAlmostEqual(report['manual_comparisons']['/odom']['displacement_minus_manual_m'],.02)
            self.assertEqual(report['mcu']['instances'][0]['measured_mm_s_abcd'][0]['mean'],100.)
            self.assertEqual(report['stopping']['physical_stop_confirmed']['state'],'unknown')
            self.assertEqual(report['topic_names']['/odom'],'/test/fused')


if __name__=='__main__':unittest.main()
