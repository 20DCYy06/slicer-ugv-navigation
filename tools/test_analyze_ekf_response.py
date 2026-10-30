#!/usr/bin/env python3
"""Deterministic checks for the offline EKF analysis definitions; no ROS."""
import math
import unittest

import analyze_ekf_response as metrics


def odom(at,x=0.,y=0.,yaw=0.):
    return [at,x,y,yaw,0.,0.,0.,0.,0.]


class ResponseMetricsTests(unittest.TestCase):
    def test_dwell_reports_first_quiet_time_only_after_confirmation(self):
        rows=[[i/4,0 if i>=1 else 1] for i in range(6)]
        found=metrics.dwell(rows,0,1.25,lambda row:row[1]==0,.5,.3)
        self.assertEqual(found['state'],'observed')
        self.assertEqual(found['first_under_s'],.25)
        self.assertEqual(found['confirmed_s'],.75)
        too_short=metrics.dwell(rows,0,.5,lambda row:row[1]==0,.5,.3)
        self.assertEqual(too_short['state'],'threshold_not_reached')
        self.assertIsNone(too_short['first_under_s'])

    def test_missing_samples_cannot_join_short_quiet_runs(self):
        rows=[[0,0],[.1,0],[.8,0],[.9,0]]
        found=metrics.dwell(rows,0,.9,lambda row:True,.5,.15)
        self.assertEqual(found['state'],'missing_data')
        self.assertTrue(found['missing_data'])
        self.assertIsNone(found['confirmed_s'])
        series=metrics.Series([odom(0),odom(.3)])
        self.assertIsNone(series.at(.1))

    def test_yaw_crossing_pi_remains_continuous(self):
        series=metrics.Series([odom(0,yaw=math.pi-.1),odom(.1,yaw=-math.pi+.1)])
        self.assertAlmostEqual(series.rows[-1][3]-series.rows[0][3],.2)
        jumps=metrics.yaw_jumps(series,0,.1,.1)
        self.assertAlmostEqual(jumps['change_over_interval_rad']['max_abs'],.2)
        self.assertAlmostEqual(series.at(.05)[3],math.pi)

    def test_lio_increment_comparison_removes_pose_origin_offsets(self):
        base=metrics.Series([odom(0),odom(.1,x=1.,y=.2,yaw=.2)])
        # A different translation and heading origin expresses the same motion.
        reference=metrics.Series([odom(0,x=5.,y=3.,yaw=math.pi/2),
            odom(.1,x=4.8,y=4.,yaw=math.pi/2+.2)])
        result=metrics.lio_increments(base,reference,0,.1)
        self.assertEqual(result['matched_samples'],2)
        self.assertLess(result['errors']['displacement_increment_m']['max_abs'],1e-12)
        self.assertLess(result['errors']['yaw_increment_rad']['max_abs'],1e-12)

    def test_stationary_window_requires_fresh_valid_and_quiet_sensors(self):
        rows=[[i/10,0.,.001,0.,True] for i in range(11)]
        valid=metrics.sensor_quiet_evidence(rows,0,1,.01,.15)
        self.assertEqual(valid['state'],'observed')
        self.assertEqual(valid['valid_quiet_fraction'],1.)
        rows[5][4]=False
        rows[6][2]=.02
        invalid=metrics.sensor_quiet_evidence(rows,0,1,.01,.15)
        self.assertEqual(invalid['state'],'incomplete_or_not_quiet')
        self.assertAlmostEqual(invalid['feedback_valid_fraction'],10/11)
        self.assertAlmostEqual(invalid['valid_quiet_fraction'],9/11)


if __name__=='__main__':
    unittest.main()
