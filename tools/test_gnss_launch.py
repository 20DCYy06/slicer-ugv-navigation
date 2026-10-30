"""Resolve GNSS/nav composition with real ROS XML; no node/device is started."""
import json
from pathlib import Path
import tempfile
import unittest

import roslaunch

ROOT = Path(__file__).resolve().parents[1]
GNSS = ROOT / 'src/ugv_bringup/launch/slicer_gnss.launch'
ONLINE = ROOT / 'src/ugv_bringup/launch/slicer_online.launch'


class GnssLaunchTests(unittest.TestCase):
    def load(self, config=None, arguments=()):
        self.assertTrue(GNSS.is_file(), 'standalone GNSS launch is missing')
        config = config or roslaunch.config.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(str(GNSS), config, argv=list(arguments), verbose=False)
        self.assertFalse(config.config_errors, config.config_errors)
        return config

    def test_receiver_starts_alone_with_one_serial_owner_and_no_tf_or_navigation(self):
        config = self.load()
        self.assertEqual([(n.package, n.type, n.name) for n in config.nodes],
                         [('cup_gnss', 'gnss_node.py', 'cup_gnss')])
        self.assertFalse(config.nodes[0].required,
                         'an optional GNSS process must not terminate composed local navigation')
        p = {k: v.value for k, v in config.params.items()}
        self.assertEqual(p['/cup_gnss/source'], 'serial')
        self.assertEqual(p['/cup_gnss/serial_port'], '/dev/gnss')
        self.assertEqual(p['/cup_gnss/expected_usb_serial'], '5185028488')
        self.assertEqual(p['/cup_gnss/frame_id'], 'gnss_link')
        self.assertFalse(p['/cup_gnss/ntrip_enabled'])
        self.assertEqual(p['/cup_gnss/max_differential_age_s'], 5.0)
        self.assertEqual(p['/cup_gnss/max_source_age_s'], 2.0)
        self.assertFalse(any('/odom' in k or '/map' in k for k in p))

    def test_existing_online_navigation_does_not_start_or_depend_on_receiver(self):
        config = roslaunch.config.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(str(ONLINE), config, verbose=False)
        self.assertFalse(any(n.package == 'cup_gnss' for n in config.nodes))

    def test_combination_preserves_unique_tf_owners_and_dwa_chain(self):
        config = roslaunch.config.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(
            str(ONLINE), config, argv=['start_sensors:=true', 'connect_control:=true'], verbose=False)
        self.load(config)
        self.assertEqual(sum(n.type == 'ekf_localization_node' for n in config.nodes), 1)
        self.assertEqual(sum(n.type == 'slam_gmapping' for n in config.nodes), 1)
        self.assertEqual(sum(n.type == 'arc_velocity_guard.py' for n in config.nodes), 1)
        self.assertEqual(sum(n.type == 'velocity_mux.py' for n in config.nodes), 1)
        self.assertEqual(sum(n.type == 'gnss_node.py' for n in config.nodes), 1)
        self.assertFalse(any(n.type in ('navsat_transform_node', 'c30d_driver_node.py')
                             for n in config.nodes))
        self.assertEqual(config.params['/slicer_ekf/odom0'].value, '/lio/odom')
        self.assertEqual(config.params['/slicer_ekf/odom1'].value, '/wheel/odom')
        self.assertEqual(config.params['/slicer_ekf/world_frame'].value, 'odom')
        self.assertEqual(config.params['/slam_gmapping/map_frame'].value, 'map')
        self.assertEqual(config.params['/ugv/move_base/base_local_planner'].value,
                         'dwa_local_planner/DWAPlannerROS')

    def test_public_config_can_be_replaced_without_embedding_credentials_in_params(self):
        # This parses a synthetic PTY/caster configuration; it never opens either.
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'TEST_ONLY.json'
            path.write_text(json.dumps({'source': 'serial', 'serial_port': '/dev/pts/TEST_ONLY',
                                        'expected_usb_serial': '', 'ntrip_enabled': True,
                                        'ntrip_host': '127.0.0.1', 'ntrip_port': 21234,
                                        'ntrip_credentials_file': '/TEST_ONLY/not-read.json'}))
            config = self.load(arguments=['config_file:=' + str(path)])
        self.assertEqual(config.params['/cup_gnss/ntrip_host'].value, '127.0.0.1')
        self.assertEqual(config.params['/cup_gnss/ntrip_port'].value, 21234)
        self.assertTrue(config.params['/cup_gnss/ntrip_enabled'].value)
        self.assertFalse(any(k.endswith(('/password', '/username')) for k in config.params))


if __name__ == '__main__':
    unittest.main(verbosity=2)
