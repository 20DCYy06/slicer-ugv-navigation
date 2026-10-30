#!/usr/bin/env python3
"""Offline replay contract tests only; no ROS master, devices or field-result claims."""
import copy
import math
from types import SimpleNamespace as NS
import unittest

import replay_ekf_response as replay


def transform(parent, child, stamp=1, xyz=(0., 0., 0.)):
    return NS(header=NS(frame_id=parent, stamp=stamp), child_frame_id=child,
              transform=NS(translation=xyz, rotation=(0., 0., 0., 1.)))


class OptionalImuReplayTests(unittest.TestCase):
    def test_covariance_override_copies_and_preserves_every_other_field(self):
        source = NS(header=NS(stamp=(11, 123), frame_id='livox_frame'),
                    angular_velocity=NS(x=.1, y=-.2, z=.3),
                    angular_velocity_covariance=[0.] * 9,
                    orientation=(0., 0., 0., 0.), orientation_covariance=[0.] * 9,
                    linear_acceleration=(0., 0., 1.), linear_acceleration_covariance=[0.] * 9)
        original = copy.deepcopy(source)
        result = replay.imu_with_variance(source, .004)
        self.assertEqual(source, original)
        self.assertIsNot(result, source)
        self.assertEqual(result.angular_velocity_covariance, [.004, 0., 0., 0., .004, 0., 0., 0., .004])
        result.angular_velocity_covariance = source.angular_velocity_covariance
        self.assertEqual(result, source)

    def test_explicit_body_frame_correction_preserves_raw_gyro_and_stamp(self):
        source = NS(header=NS(stamp=(11, 123), frame_id='livox_frame'),
                    angular_velocity=NS(x=.008, y=-.007, z=.035),
                    angular_velocity_covariance=[0.] * 9)
        original = copy.deepcopy(source)
        result = replay.imu_with_variance(source, .004, frame='body')
        self.assertEqual(source, original)
        self.assertEqual(result.header.frame_id, 'body')
        self.assertEqual(result.header.stamp, source.header.stamp)
        self.assertEqual(result.angular_velocity, source.angular_velocity)
        self.assertEqual(result.angular_velocity.z, .035)
        with self.assertRaises(ValueError):
            replay.imu_with_variance(NS(header=NS(frame_id='other_imu')), .004, frame='body')
        with self.assertRaises(ValueError):
            replay.imu_with_variance(source, .004, frame='unproved_frame')

    def test_unknown_or_invalid_gyro_variance_is_rejected(self):
        for variance in (None, 0., -1., math.nan, math.inf):
            with self.subTest(variance=variance), self.assertRaises(ValueError):
                replay.imu_with_variance(NS(), variance)

    def test_only_candidate_counts_as_imu_subscriber(self):
        base = dict(odom0='/lio/odom', odom0_config=[True], odom1='/wheel/odom', odom1_config=[True])
        profiles = dict(baseline=base, q02=dict(base), imu=dict(base, imu0='/livox/imu', imu0_config=[False]*11+[True]+[False]*3))
        self.assertEqual(replay.sensor_subscribers(profiles), {'/lio/odom': 3, '/wheel/odom': 3, '/livox/imu': 1})
        self.assertEqual(replay.sensor_subscribers({'baseline': base}), {'/lio/odom': 1, '/wheel/odom': 1, '/livox/imu': 0})

    def test_imu_candidate_cannot_enable_orientation_or_acceleration(self):
        for enabled in (5, 12):
            mask = [False] * 15
            mask[enabled] = True
            with self.subTest(enabled=enabled), self.assertRaises(ValueError):
                replay.sensor_subscribers({'bad': dict(imu0='/livox/imu', imu0_config=mask)})

    def test_multiple_static_publishers_are_aggregated_without_stamp_changes(self):
        a = transform('base_link', 'laser_link', stamp=123)
        b = transform('laser_link', 'livox_frame', stamp=456)
        repeated = transform('base_link', 'laser_link', stamp=789)
        result = replay.recorded_static_transforms([NS(transforms=[a]), NS(transforms=[b, repeated])])
        self.assertEqual(result, [a, b])
        self.assertEqual([t.header.stamp for t in result], [123, 456])

    def test_conflicting_recorded_static_geometry_is_rejected(self):
        a = transform('base_link', 'laser_link')
        for bad in (transform('other_parent', 'laser_link'), transform('base_link', 'laser_link', xyz=(1., 0., 0.))):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                replay.recorded_static_transforms([NS(transforms=[a, bad])])


if __name__ == '__main__':
    unittest.main()
