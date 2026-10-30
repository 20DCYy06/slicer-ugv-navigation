#!/usr/bin/env python3
"""TEST_ONLY depth regression checks; no ROS master, transport or devices.

Geometry fixtures retain the teammate module's original projection/ground cases.
The node adapter executes the real process_latest and geometry functions; only
ROS clocks, logging, message construction and publishing use local substitutes.
"""
import importlib.util
import math
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest.mock import patch
import warnings

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
import depth_geometry as geometry


class Stamp:
    now_seconds = 100.0

    def __init__(self, seconds=0.0):
        self.seconds = float(seconds)

    @classmethod
    def now(cls):
        return cls(cls.now_seconds)

    def to_sec(self):
        return self.seconds

    def __sub__(self, other):
        return Stamp(self.seconds - other.seconds)

    def __eq__(self, other):
        return isinstance(other, Stamp) and self.seconds == other.seconds


class Message(types.SimpleNamespace):
    FLOAT32 = 7


class Output:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)

    def sendTransform(self, message):
        self.messages.append(message)


def load_node_without_ros():
    """Import definitions only; constructing a live node remains unavailable."""
    modules = {}
    for name in ('rospy', 'tf', 'tf.transformations', 'tf2_ros',
                 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg',
                 'sensor_msgs', 'sensor_msgs.msg', 'std_msgs', 'std_msgs.msg'):
        modules[name] = types.ModuleType(name)
    modules['rospy'].Time = Stamp
    modules['rospy'].logwarn_throttle = lambda *args: None
    modules['rospy'].loginfo_throttle = lambda *args: None
    modules['tf'].Exception = type('TransformError', (Exception,), {})

    def unavailable_transform(*args):
        raise AssertionError('Unexpected uncached TF computation in node fixture')

    for name in ('quaternion_matrix', 'quaternion_from_euler',
                 'quaternion_from_matrix', 'euler_from_matrix'):
        setattr(modules['tf.transformations'], name, unavailable_transform)
    for module, names in (
        ('geometry_msgs.msg', ('TransformStamped',)),
        ('nav_msgs.msg', ('Odometry',)),
        ('sensor_msgs.msg', ('Image', 'CameraInfo', 'PointCloud2', 'PointField')),
        ('std_msgs.msg', ('Header',)),
    ):
        for name in names:
            setattr(modules[module], name, Message)
    spec = importlib.util.spec_from_file_location(
        '_ugv_depth_obstacles_test', SCRIPTS / 'depth_obstacles.py')
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    # Keep the actual matrix; replace only packing it into a ROS TF message.
    module.tf_message = lambda parent, child, matrix, stamp: Message(
        parent=parent, child=child, matrix=matrix.copy(), stamp=stamp)
    return module


NODE = load_node_without_ros()


def optical_mount():
    transform = np.eye(4)
    transform[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    transform[:3, 3] = [.14, 0., .10]
    return transform


def make_node(depth_stamp=99.75, gravity_stamp=99.75, now=100., last=None,
              depth=None, fx=100., fy=100., cx=1.5, cy=1.5):
    Stamp.now_seconds = now
    if depth is None:
        depth = np.ones((4, 4), dtype='<f4')
    depth = np.asarray(depth, dtype='<f4')
    height, width = depth.shape
    header = Message(stamp=Stamp(depth_stamp), frame_id='camera_depth_optical_frame')
    node = NODE.DepthObstacles.__new__(NODE.DepthObstacles)
    node.p = dict(gravity_max_age_s=.5, depth_scale=.001, pixel_stride=1,
                  min_depth_m=.2, max_depth_m=3.5, min_obstacle_height=.08,
                  max_obstacle_height=1.2, voxel_size_m=.05,
                  base_frame='base_link', output_frame='online_depth_optical')
    node.lock = threading.Lock()
    node.info = Message(
        header=header, width=width, height=height,
        P=[fx, 0., cx, 0., 0., fy, cy, 0., 0., 0., 1., 0.],
        K=[fx, 0., cx, 0., fy, cy, 0., 0., 1.])
    node.latest_depth = Message(
        header=header, width=width, height=height, encoding='32FC1',
        is_bigendian=False, step=width * 4, data=depth.tobytes())
    node.tilt = np.eye(4)
    node.gravity_stamp = Stamp(gravity_stamp)
    node.processed_stamp = None if last is None else Stamp(last)
    node.camera_to_base = optical_mount()
    node.hits, node.rays, node.dynamic = Output(), Output(), Output()
    return node


class GeometryChecks(unittest.TestCase):
    def test_projection_rejects_invalid_depth_and_preserves_metric_geometry(self):
        depth = np.array([[0., np.nan, 1.], [.1, 2., 4.]])
        points = geometry.project(depth, 100., 100., 1., 0., 1, .2, 3.5)
        np.testing.assert_allclose(points, [[.01, 0., 1.], [0., .02, 2.]])

    def test_subsample_before_unit_conversion_preserves_projection(self):
        raw = np.arange(35 * 47, dtype=np.uint16).reshape(35, 47)
        full = geometry.project(raw.astype(np.float32) * .001,
                                100., 101., 23.4, 17.6, 4, .2, 3.5)
        sampled = geometry.project(raw[::4, ::4].astype(np.float32) * .001,
                                   25., 25.25, 23.4 / 4, 17.6 / 4, 1, .2, 3.5)
        np.testing.assert_allclose(sampled, full)

    def test_ground_removed_under_tilt_but_retained_for_clearing(self):
        rx, ry, rz = .2, -.25, .1
        x = np.array([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)],
                      [0, math.sin(rx), math.cos(rx)]])
        y = np.array([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0],
                      [-math.sin(ry), 0, math.cos(ry)]])
        z = np.array([[math.cos(rz), -math.sin(rz), 0],
                      [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])
        transform = np.eye(4)
        transform[:3, :3] = z @ y @ x
        transform[:3, 3] = [.14, 0., .10]
        level = np.array([[.5, 0., 0.], [1., .1, .03],
                          [.6, 0., .13], [1., .1, 1.3]])
        optical = (level - transform[:3, 3]) @ transform[:3, :3]
        hits, rays = geometry.filter_points(optical, transform, .08, 1.2, .01)
        self.assertEqual((len(hits), len(rays)), (1, 4))
        np.testing.assert_allclose(hits[0], optical[2])

    def test_empty_cloud_and_voxels_do_not_invent_points(self):
        self.assertEqual(geometry.voxel(np.empty((0, 3)), .05).shape, (0, 3))
        result = geometry.voxel(np.array([[.011, 0., 0.], [.012, 0., 0.],
                                         [.11, 0., 0.]]), .05)
        np.testing.assert_allclose(result, [[.011, 0., 0.], [.11, 0., 0.]])

    def test_valid_ground_has_empty_hits_and_nonempty_clearing(self):
        points = np.array([[0., .1, .5], [.03, .1, .7]])
        hits, rays = geometry.filter_points(points, optical_mount(), .08, 1.2, .05)
        self.assertEqual(hits.shape, (0, 3))
        np.testing.assert_allclose(rays, points)

    def test_projection_rejects_nonfinite_intrinsics(self):
        for index in range(4):
            for invalid in (np.nan, np.inf, -np.inf):
                intrinsics = [100., 100., 0., 0.]
                intrinsics[index] = invalid
                with self.subTest(index=index, invalid=invalid):
                    with self.assertRaises(ValueError):
                        geometry.project(np.ones((1, 1)), *intrinsics, 1, .2, 3.5)

    def test_projection_rejects_nonpositive_focal_lengths(self):
        for index in (0, 1):
            for invalid in (0., -1.):
                intrinsics = [100., 100., 0., 0.]
                intrinsics[index] = invalid
                with self.subTest(index=index, invalid=invalid):
                    with warnings.catch_warnings():
                        warnings.simplefilter('ignore', RuntimeWarning)
                        with self.assertRaises(ValueError):
                            geometry.project(np.ones((1, 1)), *intrinsics, 1, .2, 3.5)

    def test_frame_time_boundaries_and_order(self):
        check = getattr(geometry, 'valid_frame_times', None)
        self.assertTrue(callable(check), 'A pure frame-time validator is required')
        cases = [
            ('current', 99.75, 99.75, None, True),
            ('old_boundary', 99.5, 99.5, None, True),
            ('future_boundary', 100.5, 100.5, None, True),
            ('alignment_boundary', 99.5, 100., None, True),
            ('old', 99.499, 99.499, None, False),
            ('both_future', 100.501, 100.501, None, False),
            ('gravity_old', 99.75, 99.25, None, False),
            ('gravity_future', 100.25, 100.75, None, False),
            ('misaligned', 99.5, 100.5, None, False),
            ('duplicate', 99.75, 99.75, 99.75, False),
            ('out_of_order', 99.75, 99.75, 99.8, False),
            ('newer', 99.75, 99.75, 99.5, True),
        ]
        for name, depth, gravity, last, expected in cases:
            with self.subTest(name=name):
                self.assertEqual(check(depth, gravity, 100., .5, last), expected)


class ProcessLatestChecks(unittest.TestCase):
    def assert_no_output(self, node):
        before = node.processed_stamp
        node.process_latest(None)
        self.assertEqual(len(node.hits.messages), 0)
        self.assertEqual(len(node.rays.messages), 0)
        self.assertEqual(len(node.dynamic.messages), 0)
        self.assertEqual(node.processed_stamp, before)

    def test_future_image_and_gravity_do_not_publish(self):
        self.assert_no_output(make_node(depth_stamp=105., gravity_stamp=105.))

    def test_out_of_order_image_does_not_publish(self):
        self.assert_no_output(make_node(depth_stamp=100.2, gravity_stamp=100.3,
                                        now=100.4, last=100.3))

    def test_duplicate_image_does_not_publish(self):
        self.assert_no_output(make_node(last=99.75))

    def test_stale_or_misaligned_inputs_do_not_publish(self):
        for depth, gravity in ((99., 99.), (99.75, 98.75),
                               (99.75, 99.25), (100.25, 100.75)):
            with self.subTest(depth=depth, gravity=gravity):
                self.assert_no_output(make_node(depth_stamp=depth, gravity_stamp=gravity))

    def test_nonfinite_camera_info_does_not_publish(self):
        for intrinsics in ({'cx': np.nan}, {'fy': np.nan}, {'cy': np.inf}):
            with self.subTest(intrinsics=intrinsics):
                with warnings.catch_warnings():
                    warnings.simplefilter('ignore', RuntimeWarning)
                    self.assert_no_output(make_node(**intrinsics))

    def test_time_boundary_frames_keep_original_image_stamp(self):
        for stamp in (99.5, 100.5):
            with self.subTest(stamp=stamp):
                node = make_node(depth_stamp=stamp, gravity_stamp=stamp)
                node.process_latest(None)
                self.assertEqual(len(node.hits.messages), 1)
                self.assertEqual(node.hits.messages[0].header.stamp, Stamp(stamp))
                self.assertEqual(node.rays.messages[0].header.stamp, Stamp(stamp))
                self.assertEqual(node.dynamic.messages[0].stamp, Stamp(stamp))
                self.assertEqual(node.processed_stamp, Stamp(stamp))

    def test_ground_only_frame_publishes_empty_hits_and_valid_clearing(self):
        # y_optical=0.1 m on every row: z_base=0.1-y_optical=0.
        depth = np.repeat((1. / np.arange(1, 5))[:, None], 4, axis=1)
        node = make_node(depth=depth, fx=10., fy=10., cy=-1.)
        node.process_latest(None)
        self.assertEqual(len(node.hits.messages), 1)
        self.assertEqual(node.hits.messages[0].width, 0)
        self.assertEqual(len(node.rays.messages), 1)
        rays = node.rays.messages[0]
        self.assertGreater(rays.width, 0)
        points = np.frombuffer(rays.data, dtype='<f4').reshape(-1, 3)
        self.assertTrue(np.isfinite(points).all())
        level = points @ optical_mount()[:3, :3].T + optical_mount()[:3, 3]
        np.testing.assert_allclose(level[:, 2], 0., atol=1e-7)
        self.assertEqual(node.processed_stamp, Stamp(99.75))

    def test_process_latest_executes_the_real_time_helper(self):
        check = getattr(geometry, 'valid_frame_times', None)
        self.assertTrue(callable(check), 'A pure frame-time validator is required')
        calls = []
        previous = sys.getprofile()

        def observe(frame, event, arg):
            if event == 'call' and frame.f_code is check.__code__:
                calls.append(True)

        node = make_node()
        try:
            sys.setprofile(observe)
            node.process_latest(None)
        finally:
            sys.setprofile(previous)
        self.assertTrue(calls, 'process_latest must execute the production time helper')
        self.assertEqual(len(node.rays.messages), 1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
