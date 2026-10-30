#!/usr/bin/env python3
"""Check a real Noetic install by lookup, import and XML parsing only.

Run after sourcing only Noetic and install/setup.bash; the optional read-only
teammate source path must be on ROS_PACKAGE_PATH. No ROS nodes are started.
"""
import argparse
import hashlib
import importlib
import json
import os
from pathlib import Path
import runpy
import subprocess
import sys

import roslib.packages
import roslaunch


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def package_path(package):
    path = Path(subprocess.check_output(
        ['rospack', 'find', package], text=True).strip()).resolve()
    print('PACKAGE {} {}'.format(package, path), flush=True)
    return path


def check_install(prefix):
    for package in ('ugv_bringup', 'ugv_navigation', 'ugv_slicer_control', 'cup_gnss'):
        expected = prefix / 'share' / package
        require(package_path(package) == expected,
                '{} must resolve from {}'.format(package, expected))

    entries = {
        'ugv_navigation': ('goal.py', 'depth_obstacles.py'),
        'ugv_slicer_control': ('velocity_mux.py', 'arc_velocity_guard.py',
                               'control.py', 'c30d_driver_node.py'),
        'cup_gnss': ('gnss_node.py', 'gnss_status.py'),
    }
    for package, scripts in entries.items():
        directory = prefix / 'lib' / package
        sys.path.insert(0, str(directory))
        for script in scripts:
            expected = directory / script
            found = roslib.packages.find_node(package, script) or []
            require(expected in [Path(path).resolve() for path in found],
                    '{} is not discoverable in {}'.format(script, directory))
            require(os.access(str(expected), os.X_OK),
                    '{} is not executable'.format(expected))
            # These entrypoints initialize nodes/devices only in their main block.
            runpy.run_path(str(expected), run_name='install_import_check')
            print('IMPORT {}'.format(expected), flush=True)

    helpers = {
        'ugv_navigation': ('depth_geometry',),
        'ugv_slicer_control': ('control_core', 'slicer_manual', 'navigation_core',
                               'c30d_link', 'chassis_core', 'serial_transport'),
    }
    for package, modules in helpers.items():
        for name in modules:
            module = importlib.import_module(name)
            expected = prefix / 'lib' / package / (name + '.py')
            require(Path(module.__file__).resolve() == expected,
                    '{} came from {} instead of {}'.format(
                        name, module.__file__, expected))
            print('HELPER {}'.format(expected), flush=True)

    # catkin_python_setup installs the GNSS Python library independently of
    # its executable entrypoints. Importing never starts a source or caster.
    gnss_package = importlib.import_module('cup_gnss')
    gnss_directory = Path(gnss_package.__file__).resolve().parent
    require(prefix in gnss_directory.parents,
            'cup_gnss Python package must resolve from the install prefix')
    for name in ('core', 'sources', 'quality', 'upstream_nmea', 'upstream_ntrip', 'ntrip', 'enu'):
        module = importlib.import_module('cup_gnss.' + name)
        require(Path(module.__file__).resolve().parent == gnss_directory,
                'GNSS helper did not come from install: ' + name)
        print('GNSS_HELPER {}'.format(module.__file__), flush=True)
    manifest = json.loads((gnss_directory / 'NMEA_UPSTREAM.json').read_text())
    for name, digest in manifest['files'].items():
        require(hashlib.sha256((gnss_directory / name).read_bytes()).hexdigest() == digest,
                'Installed locked NMEA source mismatch: ' + name)
    print('GNSS_NMEA_MANIFEST upstream={}'.format(manifest['version']), flush=True)

    bringup = prefix / 'share' / 'ugv_bringup'
    for name in ('slicer_gnss.yaml', 'gnss_site.yaml', 'gnss_enu_origin.json'):
        path = bringup / 'config' / name
        require(path.is_file(), 'Missing installed GNSS config {}'.format(path))
        print('GNSS_CONFIG {}'.format(path), flush=True)
    frozen = (
        'depth_camera.yaml', 'depth_costmap.yaml', 'gmapping_online.yaml',
        'online_runtime/config/slicer_config.yaml',
        'online_runtime/.generated/MID360.json',
        'online_runtime/.generated/fusion.yaml',
        'online_runtime/.generated/lio.yaml',
        'online_runtime/.generated/scan.yaml',
        'online_runtime/.generated/sensor_launch.yaml',
    )
    for relative in frozen:
        path = bringup / 'config' / relative
        require(path.is_file(), 'Missing frozen config {}'.format(path))
        print('CONFIG {}'.format(path), flush=True)

    # Package discovery is distinct from availability of compiled sensor nodes.
    for package in ('slicer_bringup', 'slicer_wasd_control', 'slicer_navigation',
                    'fast_lio', 'livox_ros_driver2', 'gmapping',
                    'realsense2_camera', 'robot_localization',
                    'pointcloud_to_laserscan', 'ntrip_client', 'nmea_navsat_driver'):
        package_path(package)
    for package, script in (
            ('slicer_bringup', 'sensor_geometry.py'),
            ('slicer_bringup', 'lio_projector.py'),
            ('slicer_navigation', 'arm_cloud_filter.py'),
            ('slicer_navigation', 'self_scan_filter.py')):
        found = roslib.packages.find_node(package, script) or []
        print('EXTERNAL_SCRIPT {}/{} {}'.format(
            package, script, found or 'UNAVAILABLE (source reference has no executable node)'),
            flush=True)
    for package, node in (('fast_lio', 'fastlio_mapping'),
                          ('livox_ros_driver2', 'livox_ros_driver2_node')):
        found = roslib.packages.find_node(package, node) or []
        print('EXTERNAL_COMPILED {}/{} {}'.format(
            package, node, found or 'UNAVAILABLE (source reference only; not built here)'),
            flush=True)

    launch = bringup / 'launch' / 'slicer_online.launch'
    branches = (
        ('default', []),
        ('guard', ['connect_control:=true']),
        ('sensors', ['start_sensors:=true']),
        ('camera', ['start_camera:=true', 'camera_serial:=install-check-d455']),
        ('no_depth', ['enable_depth:=false']),
    )
    for name, arguments in branches:
        config = roslaunch.config.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(
            str(launch), config, argv=arguments, verbose=False)
        require(not config.config_errors, str(config.config_errors))
        require(not any(n.type == 'c30d_driver_node.py' for n in config.nodes),
                'Online launch unexpectedly includes the chassis driver')
        if name == 'sensors':
            actual = config.params['/slicer_geometry/config_file'].value
            expected = bringup / 'config/online_runtime/config/slicer_config.yaml'
            require(Path(actual).resolve() == expected,
                    'Sensor config did not resolve from install: {}'.format(actual))
        print('PARSE {} nodes={} (none started)'.format(name, len(config.nodes)), flush=True)
    gnss_config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(
        str(bringup / 'launch/slicer_gnss.launch'), gnss_config, verbose=False)
    require(not gnss_config.config_errors, str(gnss_config.config_errors))
    require([(n.package, n.type) for n in gnss_config.nodes] == [('cup_gnss', 'gnss_node.py')],
            'Standalone GNSS must not start navigation, a second serial owner or localization TF')
    print('PARSE gnss nodes=1 (none started)', flush=True)
    print('INSTALL CHECK PASSED: lookup/import/parse only; no hardware verification', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True,
                        help='The catkin_make install prefix to verify')
    args = parser.parse_args()
    check_install(args.prefix.resolve())


if __name__ == '__main__':
    main()
