#!/usr/bin/env python3
"""Resolve launch files with the official ROS loader; never start ROS nodes."""
import os
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

try:
    import roslaunch
    from rosgraph.names import ns_join
except ImportError:
    roslaunch = None

ROOT = Path(__file__).resolve().parents[1]
BRINGUP = ROOT / 'src/ugv_bringup'
ONLINE = BRINGUP / 'launch/slicer_online.launch'


@unittest.skipIf(roslaunch is None, 'requires the official Noetic roslaunch parser')
class OnlineLaunchTests(unittest.TestCase):
    def setUp(self):
        self.assertTrue(ONLINE.is_file(), 'the composable online entry must exist')

    def load(self, arguments=(), path=ONLINE):
        self.assertTrue(path.is_file(), 'the composable online entry must exist')
        config = roslaunch.config.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(
            str(path), config, argv=list(arguments), verbose=False)
        self.assertFalse(config.config_errors, config.config_errors)
        return config

    @staticmethod
    def node(config, name):
        return next(n for n in config.nodes if ns_join(n.namespace, n.name) == name)

    def test_default_planning_uses_online_map_and_continuous_local_frame(self):
        config = self.load()
        params = {key: value.value for key, value in config.params.items()}
        prefix = '/ugv/move_base/'
        self.assertEqual(params[prefix + 'global_costmap/global_frame'], 'map')
        self.assertEqual(params[prefix + 'local_costmap/global_frame'], 'odom')
        self.assertFalse(params[prefix + 'global_costmap/rolling_window'])
        self.assertTrue(params[prefix + 'local_costmap/rolling_window'])
        self.assertEqual(params[prefix + 'global_costmap/static_layer/map_topic'], '/map')
        self.assertFalse(params[prefix + 'global_costmap/static_layer/first_map_only'])
        self.assertTrue(params[prefix + 'GlobalPlanner/allow_unknown'])
        self.assertTrue(params[prefix + 'local_costmap/track_unknown_space'])
        self.assertTrue(params[prefix + 'local_costmap/always_send_full_costmap'])
        planner = self.node(config, '/ugv/move_base')
        target = dict(planner.remap_args)['cmd_vel']
        self.assertEqual(ns_join(planner.namespace, target), '/ugv/cmd_vel/nav_raw')
        self.assertFalse(any(n.type in ('c30d_driver_node.py', 'velocity_mux.py',
                                       'arc_velocity_guard.py') for n in config.nodes))

    def test_control_connection_has_one_guard_mux_and_no_driver(self):
        config = self.load(['connect_control:=true'])
        names = [ns_join(n.namespace, n.name) for n in config.nodes]
        self.assertEqual(len(names), len(set(names)))
        self.assertEqual(sum(n.type == 'arc_velocity_guard.py' for n in config.nodes), 1)
        self.assertEqual(sum(n.type == 'velocity_mux.py' for n in config.nodes), 1)
        self.assertFalse(any(n.type == 'c30d_driver_node.py' for n in config.nodes))
        self.assertEqual(dict(self.node(config, '/ugv/move_base').remap_args)['cmd_vel'],
                         '/cmd_vel/nav_raw')
        p = config.params
        self.assertEqual(p['/slicer_nav_arc_guard/input_topic'].value, '/cmd_vel/nav_raw')
        self.assertEqual(p['/slicer_nav_arc_guard/costmap_topic'].value,
                         '/ugv/move_base/local_costmap/costmap')
        self.assertEqual(p['/slicer_nav_arc_guard/output_topic'].value, '/cmd_vel/nav')
        self.assertFalse(any('control.py' == n.type for n in config.nodes))

    def test_sensors_preserve_fusion_and_unique_tf_owners(self):
        config = self.load(['start_sensors:=true'])
        self.assertEqual(sum(n.type == 'ekf_localization_node' for n in config.nodes), 1)
        self.assertEqual(sum(n.type == 'slam_gmapping' for n in config.nodes), 1)
        p = config.params
        self.assertEqual(p['/slam_gmapping/map_frame'].value, 'map')
        self.assertEqual(p['/slam_gmapping/odom_frame'].value, 'odom')
        self.assertEqual(p['/slicer_ekf/world_frame'].value, 'odom')
        self.assertTrue(p['/slicer_ekf/publish_tf'].value)
        self.assertEqual([i for i, v in enumerate(p['/slicer_ekf/odom0_config'].value) if v],
                         [0, 1, 5])
        self.assertEqual([i for i, v in enumerate(p['/slicer_ekf/odom1_config'].value) if v],
                         [6])
        self.assertAlmostEqual(p['/slicer_ekf/process_noise_covariance'].value[11*15+11], .2)
        self.assertFalse(p['/publish/tf_en'].value)
        self.assertFalse(any(n.type in ('amcl', 'map_server', 'c30d_driver_node.py')
                             for n in config.nodes))

    def test_depth_reuses_independent_marking_and_clearing_layers(self):
        config = self.load()
        self.assertEqual(self.node(config, '/online_depth_obstacles').package, 'ugv_navigation')
        p = config.params
        for scope in ('global_costmap', 'local_costmap'):
            base = '/ugv/move_base/' + scope + '/'
            layers = p[base + 'plugins'].value
            self.assertEqual(sum(x['type'] == 'costmap_2d::InflationLayer' for x in layers), 1)
            self.assertEqual(layers[-1]['type'], 'costmap_2d::InflationLayer')
            self.assertIn('depth_layer', [x['name'] for x in layers])
            self.assertTrue(p[base + 'depth_layer/depth_hits/marking'].value)
            self.assertFalse(p[base + 'depth_layer/depth_hits/clearing'].value)
            self.assertFalse(p[base + 'depth_layer/depth_rays/marking'].value)
            self.assertTrue(p[base + 'depth_layer/depth_rays/clearing'].value)
            self.assertEqual(p[base + 'depth_layer/depth_hits/topic'].value,
                             '/online_depth/obstacles')

    def test_disabling_depth_removes_processor_and_observation_sources(self):
        config = self.load(['enable_depth:=false'])
        self.assertFalse(any(n.type == 'depth_obstacles.py' for n in config.nodes))
        for scope in ('global_costmap', 'local_costmap'):
            layers = config.params['/ugv/move_base/' + scope + '/plugins'].value
            self.assertNotIn('depth_layer', [x['name'] for x in layers])

    def test_current_navigation_limits_and_footprint_survive_online_overlay(self):
        for planner, key in [('dwa', 'DWAPlannerROS'), ('teb', 'TebLocalPlannerROS')]:
            with self.subTest(planner=planner):
                config = self.load(['local_planner:=' + planner])
                p = config.params
                prefix = '/ugv/move_base/'
                self.assertAlmostEqual(p[prefix + key + '/max_vel_x'].value, .125)
                self.assertAlmostEqual(p[prefix + key + '/max_vel_theta'].value, .25)
                self.assertAlmostEqual(p[prefix + key + '/xy_goal_tolerance'].value, .10)
                self.assertAlmostEqual(p[prefix + key + '/yaw_goal_tolerance'].value, .12)
                for scope in ('global_costmap', 'local_costmap'):
                    self.assertEqual(p[prefix + scope + '/footprint'].value,
                                     [[-.165, -.14], [-.165, .14], [.27, .14], [.27, -.14]])
                    self.assertAlmostEqual(p[prefix + scope + '/inflation_layer/inflation_radius'].value,
                                           .24)

    def test_custom_namespace_and_external_mux_use_matching_guard_topics(self):
        config = self.load(['namespace:=rescue', 'connect_control:=true', 'start_mux:=false'])
        self.node(config, '/rescue/move_base')
        self.assertFalse(any(n.type == 'velocity_mux.py' for n in config.nodes))
        self.assertEqual(config.params['/slicer_nav_arc_guard/planner_namespace'].value,
                         '/rescue/move_base')
        self.assertEqual(config.params['/slicer_nav_arc_guard/costmap_topic'].value,
                         '/rescue/move_base/local_costmap/costmap')

    def test_far_field_target_fits_initial_map_and_extent_can_be_reduced(self):
        # 40 x 40 m at arbitrary heading can put a far corner over 55 m along
        # one map axis. allow_unknown alone cannot fix an out-of-grid target.
        config = self.load()
        p = config.params
        for axis in ('x', 'y'):
            self.assertLess(p['/slam_gmapping/' + axis + 'min'].value, -55.0)
            self.assertGreater(p['/slam_gmapping/' + axis + 'max'].value, 55.0)
        smaller = self.load(['initial_map_half_extent:=25'])
        self.assertEqual(smaller.params['/slam_gmapping/xmin'].value, -25.0)
        self.assertEqual(smaller.params['/slam_gmapping/ymax'].value, 25.0)

    def test_camera_serial_and_sensor_flags_reach_real_driver_include(self):
        config = self.load(['start_camera:=true', 'camera_serial:=example-d455'])
        values = {k: v.value for k, v in config.params.items()}
        serials = [v for k, v in values.items() if k.endswith('/serial_no')]
        self.assertEqual(serials, ['example-d455'])
        self.assertFalse(any(v for k, v in values.items()
                             if k.endswith('/enable_gyro') or k.endswith('/enable_accel')))
        self.assertEqual(sum(n.type == 'depth_obstacles.py' for n in config.nodes), 1)

    def test_installed_share_layout_resolves_without_source_runtime_directory(self):
        # Model the declared CMake install(DIRECTORY launch config ...) layout.
        # Only resources are staged; this does not claim catkin compiled anything.
        with tempfile.TemporaryDirectory() as tmp:
            share = Path(tmp) / 'share'
            for package in ('ugv_bringup', 'ugv_navigation', 'ugv_slicer_control'):
                source = ROOT / 'src' / package
                target = share / package
                target.mkdir(parents=True)
                shutil.copy2(source / 'package.xml', target / 'package.xml')
                for folder in ('launch', 'config'):
                    if (source / folder).is_dir():
                        shutil.copytree(source / folder, target / folder)
            ros_paths = os.environ.get('ROS_PACKAGE_PATH', '').split(os.pathsep)
            paths = [str(share)] + [p for p in ros_paths if p != str(ROOT / 'src')]
            # $(find package)/file checks Catkin prefixes before ROS_PACKAGE_PATH.
            # Exclude the real build/install so this staged share is the only
            # copy of our packages visible in the simulated install layout.
            own_prefixes = {ROOT / 'devel', ROOT / 'install'}
            prefixes = [p for p in os.environ.get('CMAKE_PREFIX_PATH', '').split(os.pathsep)
                        if p and Path(p).resolve() not in own_prefixes]
            with patch.dict(os.environ, {
                    'ROS_PACKAGE_PATH': os.pathsep.join(paths),
                    'CMAKE_PREFIX_PATH': os.pathsep.join(prefixes)}):
                # The loader caches package lookup across loads; start a fresh lookup.
                with patch.object(roslaunch.substitution_args, '_rospack', None):
                    config = self.load(['start_sensors:=true'],
                                       share / 'ugv_bringup/launch/slicer_online.launch')
            sensor_config = config.params['/slicer_geometry/config_file'].value
            self.assertTrue(str(sensor_config).startswith(str(share)))
            self.assertTrue(Path(sensor_config).is_file())


if __name__ == '__main__':
    unittest.main(verbosity=2)
