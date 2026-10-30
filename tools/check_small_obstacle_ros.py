#!/usr/bin/env python3
"""TEST_ONLY: actual teammate filters and standard projection, synthetic clouds.

Run only in a no-device Docker --network none container. Uses a private master,
the read-only 2026-10-26 source snapshot, and the new diagnostic projection.
No LIO, EKF, move_base, driver, goal, velocity, or real sensor is started.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import traceback
import xmlrpc.client


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='New TEST_ONLY evidence directory')
    parser.add_argument('--source-root', type=Path, default=root / 'deliverables' /
                        'slicer_odom_review_20261026/external_source/Slicer_Workspace')
    args = parser.parse_args()
    require(Path('/.dockerenv').is_file(), 'Requires Docker')
    require({p.name for p in Path('/sys/class/net').iterdir()} == {'lo'},
            'Requires Docker --network none')
    require(not list(Path('/dev').glob('ttyACM*')) and not list(Path('/dev').glob('ttyUSB*')),
            'Requires no serial devices')
    source = args.source_root.resolve()
    config = source / 'config/slicer_config.yaml'
    scripts = source / 'catkin_ws/src'
    geometry = scripts / 'slicer_bringup/scripts/sensor_geometry.py'
    arm = scripts / 'slicer_navigation/scripts/arm_cloud_filter.py'
    body = scripts / 'slicer_navigation/scripts/self_scan_filter.py'
    for path in (config, geometry, arm, body):
        require(path.is_file(), 'Missing read-only source fixture: ' + str(path))
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:12429', ROS_IP='127.0.0.1',
                      ROS_HOME=str(output / 'ros'), ROS_LOG_DIR=str(output / 'roslog'),
                      PYTHONDONTWRITEBYTECODE='1')
    os.environ.pop('ROS_HOSTNAME', None)
    result = dict(TEST_ONLY=True, passed=False, cases=[],
                  boundary='Synthetic cloud/filter/projection only; no real sensing or costmap clearing claim',
                  source_sha256={str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in (config, geometry, arm, body)})
    processes, logs, subscribers = [], [], []

    def launch(name, command):
        stream = (output / (name + '.log')).open('w')
        logs.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        processes.append(process)
        return process

    started = time.monotonic()
    try:
        launch('master', ['roscore', '-p', '12429'])
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        until = time.monotonic() + 12
        while True:
            try:
                require(master.getPid('/TEST_ONLY_small_obstacle')[0] == 1, 'Master rejected getPid')
                break
            except (OSError, xmlrpc.client.Error):
                require(time.monotonic() < until, 'Private master did not start')
                time.sleep(0.1)

        import rospy
        import yaml
        from sensor_msgs.msg import LaserScan, PointCloud2
        from sensor_msgs.point_cloud2 import create_cloud_xyz32, read_points
        from std_msgs.msg import Header

        snapshot = yaml.safe_load(config.read_text())
        # Fixture admission: hand-derived body->base translation below assumes
        # this snapshot, not an arbitrary or live configuration.
        require(snapshot['sensors']['base_to_laser'] ==
                dict(xyz=[0.07, 0.0, 0.18], quaternion=[0, 0, 0, 1]), 'Fixture mounting changed')
        require(snapshot['sensors']['lidar_to_imu_translation'] == [-0.011, -0.02329, 0.04412],
                'Fixture IMU translation changed')
        require(snapshot['sensors']['lidar_to_imu_rotation'] == [1, 0, 0, 0, 1, 0, 0, 0, 1],
                'Fixture IMU rotation changed')
        rospy.init_node('TEST_ONLY_small_obstacle', disable_signals=True)
        require(not rospy.get_param('/use_sim_time', False), 'Requires wall time')
        latest, lock = {}, threading.Lock()

        def receive(stage):
            def callback(msg):
                with lock:
                    latest[stage] = msg
            return callback

        for stage, topic, kind in (
                ('filtered', '/TEST_ONLY/cloud_filtered', PointCloud2),
                ('raw', '/TEST_ONLY/scan_raw', LaserScan),
                ('scan', '/TEST_ONLY/scan', LaserScan)):
            subscribers.append(rospy.Subscriber(topic, kind, receive(stage), queue_size=5))
        publisher = rospy.Publisher('/TEST_ONLY/cloud_body', PointCloud2, queue_size=2)
        common = ['_config_file:=' + str(config)]
        launch('geometry', ['python3', str(geometry), '__name:=TEST_ONLY_geometry'] + common)
        launch('arm_filter', ['python3', str(arm), '__name:=TEST_ONLY_arm_filter',
                             '/cloud_registered_body:=/TEST_ONLY/cloud_body',
                             '/cloud_nav_filtered:=/TEST_ONLY/cloud_filtered'] + common)
        launch('projection', ['roslaunch', str(root / 'src/ugv_bringup/launch/small_obstacle_projection.launch'),
                              'namespace:=TEST_ONLY/projection',
                              'cloud_topic:=/TEST_ONLY/cloud_filtered',
                              'scan_topic:=/TEST_ONLY/scan_raw'])
        launch('body_filter', ['python3', str(body), '__name:=TEST_ONLY_body_filter',
                              '/scan_raw:=/TEST_ONLY/scan_raw', '/scan:=/TEST_ONLY/scan',
                              '/scan_front:=/TEST_ONLY/scan_front'] + common)
        until = time.monotonic() + 15
        while publisher.get_num_connections() == 0:
            require(time.monotonic() < until, 'Arm filter did not subscribe')
            require(all(p.poll() is None for p in processes), 'A node exited; inspect TEST_ONLY logs')
            time.sleep(0.05)

        def exercise(name, base_points, want_raw, want_scan, want_masked=0, want_nan=0):
            case = dict(name=name, passed=False, input_base_xyz=base_points)
            result['cases'].append(case)
            try:
                # Hand-derived from mounting plus Livox internal extrinsics:
                # base->body = (0.081, 0.02329, 0.13588), identity rotation.
                points = [(x - 0.081, y - 0.02329, z - 0.13588) for x, y, z in base_points]
                stamp = rospy.Time.now()
                msg = create_cloud_xyz32(Header(stamp=stamp, frame_id='body'), points)
                deadline = time.monotonic() + 5
                while True:
                    require(all(p.poll() is None for p in processes), 'A node exited; inspect TEST_ONLY logs')
                    publisher.publish(msg)
                    time.sleep(0.10)
                    with lock:
                        got = {key: value for key, value in latest.items()
                               if value.header.stamp == stamp}
                    if all(key in got for key in ('filtered', 'raw', 'scan')):
                        break
                    require(time.monotonic() < deadline, 'Missing output at a filter boundary')
                finite_raw = sorted(r for r in got['raw'].ranges if math.isfinite(r))
                finite_scan = sorted(r for r in got['scan'].ranges if math.isfinite(r))
                masked = sum(not all(math.isfinite(v) for v in point) for point in
                             read_points(got['filtered'], field_names=('x', 'y', 'z'), skip_nans=False))
                nan_count = sum(math.isnan(r) for r in got['scan'].ranges)
                require(got['raw'].header.frame_id == got['scan'].header.frame_id == 'base_link',
                        'Projection did not transform into base_link')
                for stage, actual, expected in (('raw', finite_raw, want_raw), ('scan', finite_scan, want_scan)):
                    require(len(actual) == len(expected) and all(abs(a - e) < 1e-4
                            for a, e in zip(actual, expected)),
                            '{} finite ranges {} != {}'.format(stage, actual, expected))
                require(masked == want_masked, 'Unexpected masked cloud point count: ' + str(masked))
                require(nan_count == want_nan, 'Unexpected body-mask NaN count: ' + str(nan_count))
                require(all(math.isinf(r) and r > 0 for r in got['raw'].ranges if not math.isfinite(r)),
                        'Empty projection bins must use positive infinity')
                case.update(passed=True, finite_raw_ranges=finite_raw, finite_scan_ranges=finite_scan,
                            masked_cloud_points=masked, body_mask_nan_bins=nan_count,
                            projected_frame=got['raw'].header.frame_id,
                            positive_inf_bins=sum(math.isinf(r) and r > 0 for r in got['raw'].ranges))
            except Exception as error:
                case['error'] = str(error)
                with (output / 'errors.log').open('a') as stream:
                    traceback.print_exc(file=stream)
            print('TEST_ONLY ' + json.dumps(case), flush=True)
            (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')

        exercise('low_returns_at_0.5_1_2m', [(0.5, 0, 0.06), (0, 1, 0.06), (0, -2, 0.06)],
                 [0.5, 1.0, 2.0], [0.5, 1.0, 2.0])
        exercise('above_min_and_below_max_height', [(1, 0, 0.041), (0, 1, 0.499)], [1, 1], [1, 1])
        exercise('below_min_and_above_max_height', [(1, 0, 0.039), (0, 1, 0.501)], [], [])
        exercise('range_min_and_max_outside', [(0.049, 0, 0.06), (0, 20.01, 0.06)], [], [])
        exercise('range_max_inside', [(0, 19.99, 0.06)], [19.99], [19.99])
        exercise('low_object_inside_arm_xy_envelope', [(0.16, 0, 0.06)], [0.16], [0.16])
        exercise('high_arm_echo_masked_before_projection', [(0.16, 0, 0.24)], [], [], want_masked=1)
        exercise('body_rectangle_masks_with_nan', [(0.12, 0, 0.06)], [0.12], [], want_nan=1)
        exercise('just_outside_body_padding', [(0.126, 0, 0.06)], [0.126], [0.126])
        exercise('nearest_return_in_same_bin', [(1, 0, 0.06), (2, 0, 0.06)], [1], [1])
        # The object disappears while a below-band ground return remains.
        exercise('removed_returns_produce_inf', [(1, 0, 0.0)], [], [])
        result['master_state'] = master.getSystemState('/TEST_ONLY_small_obstacle')[2]
        result['projection_parameters'] = rospy.get_param('/TEST_ONLY/projection/cloud_to_scan')
        result['passed'] = all(case['passed'] for case in result['cases'])
    except Exception as error:
        result['error'] = str(error)
        with (output / 'errors.log').open('a') as stream:
            traceback.print_exc(file=stream)
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        for process in reversed(processes):
            stop(process)
        for stream in logs:
            stream.close()
        result['elapsed_seconds'] = round(time.monotonic() - started, 3)
        (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print('TEST_ONLY ' + json.dumps(result, indent=2), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
