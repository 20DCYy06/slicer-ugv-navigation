#!/usr/bin/env python3
"""Source dependency and launch parsing check; never starts ROS nodes."""
from pathlib import Path
import os
import subprocess
import roslaunch
import roslib.packages

ROOT = Path(__file__).resolve().parents[1]
bringup = ROOT / 'slicer_ugv_ros1/src/ugv_bringup'
for package in ('ugv_bringup', 'ugv_navigation', 'ugv_slicer_control', 'cup_gnss',
                'slicer_bringup', 'slicer_navigation', 'slicer_wasd_control',
                'fast_lio', 'livox_ros_driver2', 'gmapping', 'realsense2_camera',
                'ntrip_client', 'nmea_navsat_driver'):
    print('PACKAGE', package, subprocess.check_output(
        ['rospack', 'find', package], text=True).strip())

for package, script in (
    ('slicer_bringup', 'sensor_geometry.py'), ('slicer_bringup', 'lio_projector.py'),
    ('slicer_navigation', 'arm_cloud_filter.py'), ('slicer_navigation', 'self_scan_filter.py'),
    ('slicer_navigation', 'uav_waypoint_receiver.py'),
):
    path = ROOT / 'Slicer_Workspace/catkin_ws/src' / package / 'scripts' / script
    assert path.is_file(), path
    found = roslib.packages.find_node(package, script) or []
    # Original non-executable Python sources were run through Catkin wrappers on
    # the Jetson. Do not alter their mode or claim those wrappers are in this ZIP.
    print('SCRIPT_SOURCE', path, 'mode', oct(path.stat().st_mode & 0o777),
          'find_node', found or 'NEEDS CATKIN WRAPPER (not included)')

for launch, arguments in (
    ('slicer_online.launch', []),
    ('slicer_online.launch', ['start_sensors:=true', 'connect_control:=true']),
    ('slicer_online.launch', ['start_camera:=true', 'camera_serial:=source-check']),
    ('slicer_gnss.launch', []),
):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(
        str(bringup / 'launch' / launch), config, argv=arguments, verbose=False)
    assert not config.config_errors, config.config_errors
    assert not any(n.type == 'c30d_driver_node.py' for n in config.nodes)
    print('PARSE', launch, arguments, 'nodes', len(config.nodes), 'NONE STARTED')

print('PASS: source presence, package lookup and launch XML parsing only; '
      'no external sensor binary build or hardware verification')
