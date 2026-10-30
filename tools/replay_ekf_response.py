#!/usr/bin/env python3
"""OFFLINE ONLY: replay recorded LIO/wheel odometry into real EKF nodes.

Run with tools/noetic.sh (Docker --network none, no devices). No Twist, dynamic
TF, Action, driver, or navigation publisher is created. Optional /livox/imu
requires an explicit wz-only imu0 profile and positive --imu-variance. Its
recorded static TF is preloaded; only its angular covariance is overridden.
An explicit --imu-frame body permits the source/config-proven livox_frame
name correction on a copy; gyro values, including bias, are unchanged.
All input stamps and measurement order are unchanged; /clock follows bag
receipt times, quantized to 10 ms. IMU and LIO share sensor information.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xmlrpc.client


def imu_with_variance(message, variance, frame=None):
    """Copy raw IMU with gyro variance in (rad/s)^2 and optional proven name correction."""
    if variance is None or not math.isfinite(variance) or variance <= 0:
        raise ValueError('IMU replay requires finite positive --imu-variance in (rad/s)^2')
    if frame is not None and (frame != 'body' or message.header.frame_id != 'livox_frame'):
        raise ValueError('Only explicit source/config-proven livox_frame -> body correction is supported')
    result = deepcopy(message)
    result.angular_velocity_covariance = [variance if i in (0, 4, 8) else 0.
                                          for i in range(9)]
    if frame is not None:
        result.header.frame_id = frame
    return result


def sensor_subscribers(effective):
    """Count EKF nodes using each supported input, including optional gyro."""
    counts = {'/lio/odom': 0, '/wheel/odom': 0, '/livox/imu': 0}
    for name, params in effective.items():
        for topic in ('/lio/odom', '/wheel/odom'):
            if any(params.get(sensor) == topic and any(params.get(sensor + '_config', []))
                   for sensor in ('odom0', 'odom1')):
                counts[topic] += 1
        if params.get('imu0'):
            if (params['imu0'] != '/livox/imu' or
                    params.get('imu0_config') != [False]*11 + [True] + [False]*3):
                raise ValueError(name + ': optional imu0 must use /livox/imu and enable only wz')
            counts['/livox/imu'] += 1
    return counts


def recorded_static_transforms(messages):
    """Aggregate recorded immutable static geometry without changing stamps."""
    by_child = {}
    for message in messages:
        for transform in message.transforms:
            child = transform.child_frame_id
            previous = by_child.get(child)
            if previous is not None:
                if (previous.header.frame_id != transform.header.frame_id or
                        previous.transform != transform.transform):
                    raise ValueError('Conflicting recorded /tf_static for ' + child)
            else:
                by_child[child] = transform
    return list(by_child.values())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--rate', type=float, default=4.)
    parser.add_argument('--profiles', type=Path, required=True,
                        help='JSON name -> private EKF parameter overrides')
    parser.add_argument('--imu-variance', type=float,
                        help='OFFLINE candidate gyro variance in (rad/s)^2; required with imu0, replaces all three angular diagonal entries')
    parser.add_argument('--imu-frame', choices=['body'],
                        help='Explicit OFFLINE livox_frame -> body name correction; requires matching source/config evidence and actual recorded body TF; no gyro or bias changes')
    args = parser.parse_args()
    if (not Path('/.dockerenv').exists() or
            {p.name for p in Path('/sys/class/net').iterdir()} != {'lo'}):
        raise RuntimeError('Use tools/noetic.sh: Docker, --network none, no devices')
    if not 0 < args.rate <= 5:
        raise ValueError('Replay rate must be in (0, 5]')
    if args.imu_variance is not None and (not math.isfinite(args.imu_variance) or args.imu_variance <= 0):
        raise ValueError('--imu-variance must be finite and positive')
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:12537', ROS_IP='127.0.0.1',
                      ROS_HOME=str(args.output / 'ros'), ROS_LOG_DIR=str(args.output / 'roslog'))
    os.environ.pop('ROS_HOSTNAME', None)
    processes, logs = [], []
    try:
        log = (args.output / 'master.log').open('w'); logs.append(log)
        core = subprocess.Popen(['roscore', '-p', '12537'], stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(core)
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        for _ in range(100):
            try:
                if master.getPid('/offline_ekf')[0] == 1:
                    break
            except (OSError, xmlrpc.client.Error):
                pass
            time.sleep(.1)
        else:
            raise RuntimeError('Private ROS master unavailable')
        import rosbag
        import rospy
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu
        rospy.set_param('/use_sim_time', True)
        rospy.init_node('offline_ekf_replay', disable_signals=True)
        profiles = json.loads(args.profiles.read_text())
        manifest = json.loads(args.bag.with_name('manifest.json').read_text())
        baseline = manifest['before']['parameters']['/slicer_ekf']
        if baseline != manifest['after']['parameters']['/slicer_ekf']:
            raise RuntimeError('EKF parameters changed during original recording')
        effective = {}
        for name, overrides in profiles.items():
            params = dict(baseline, **overrides)
            # Only suppress the replay node's TF output; estimation is unchanged.
            params['publish_tf'] = False
            params['print_diagnostics'] = False
            effective[name] = params
        expected = sensor_subscribers(effective)
        use_imu = expected['/livox/imu'] > 0
        if use_imu and args.imu_variance is None:
            raise ValueError('IMU replay requires explicit --imu-variance; recorded zeros mean unknown')
        if args.imu_frame is not None and not use_imu:
            raise ValueError('--imu-frame requires an explicit wz-only imu0 profile')
        input_topics = ['/lio/odom', '/wheel/odom']
        if use_imu:
            input_topics += ['/livox/imu', '/tf_static']
        rows, static_messages, imu_frames, raw_covariances = [], [], set(), set()
        source_imu_frames = set()
        input_counts = {topic: 0 for topic in input_topics if topic != '/tf_static'}
        with rosbag.Bag(str(args.bag)) as bag:
            begin, end = bag.get_start_time(), bag.get_end_time()
            for topic, msg, stamp in bag.read_messages(topics=input_topics):
                if topic == '/tf_static':
                    static_messages.append(msg)
                    continue
                if topic == '/livox/imu':
                    source_imu_frames.add(msg.header.frame_id)
                    raw_covariances.add(tuple(msg.angular_velocity_covariance))
                    msg = imu_with_variance(msg, args.imu_variance, args.imu_frame)
                    imu_frames.add(msg.header.frame_id)
                elif msg.header.frame_id != baseline['world_frame'] or msg.child_frame_id != baseline['base_link_frame']:
                    raise RuntimeError('Replay needs recorded input frames to match EKF world/base')
                input_counts[topic] += 1
                rows.append((stamp.to_sec(), topic, msg))
        static_transforms = recorded_static_transforms(static_messages)
        if use_imu:
            import tf2_ros
            from tf2_msgs.msg import TFMessage
            if not imu_frames or not static_transforms:
                raise RuntimeError('IMU replay needs recorded /livox/imu and actual /tf_static geometry')
            static_buffer = tf2_ros.Buffer()
            for transform in static_transforms:
                static_buffer.set_transform_static(transform, 'recorded_bag')
            for frame in imu_frames:
                if not frame or not static_buffer.can_transform(baseline['base_link_frame'], frame, rospy.Time(0)):
                    raise RuntimeError('Recorded /tf_static does not connect IMU frame ' + repr(frame) + ' to EKF base frame')
            static_publisher = rospy.Publisher('/tf_static', TFMessage, queue_size=1, latch=True)
            static_publisher.publish(TFMessage(transforms=static_transforms))
        clock = rospy.Publisher('/clock', Clock, queue_size=100)
        publishers = {t: rospy.Publisher(t, Odometry, queue_size=200)
                      for t in ['/lio/odom', '/wheel/odom']}
        if use_imu:
            publishers['/livox/imu'] = rospy.Publisher('/livox/imu', Imu, queue_size=1000)
        received = {name: [] for name in profiles}
        subscribers = []

        def capture(msg, name):
            p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            received[name].append([msg.header.stamp.to_sec(), p.x, p.y, yaw,
                                   v.linear.x, v.linear.y, v.angular.z,
                                   msg.pose.covariance[35], msg.twist.covariance[35]])

        for name, params in effective.items():
            rospy.set_param('/replay_' + name, params)
            topic = '/offline_ekf/' + name
            subscribers.append(rospy.Subscriber(topic, Odometry, capture,
                                                callback_args=name, queue_size=10000))
            log = (args.output / (name + '.log')).open('w'); logs.append(log)
            proc = subprocess.Popen(['rosrun', 'robot_localization', 'ekf_localization_node',
                                     '__name:=replay_' + name, 'odometry/filtered:=' + topic],
                                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(proc)
        for _ in range(200):
            clock.publish(Clock(rospy.Time.from_sec(begin - .1)))
            if all(p.get_num_connections() == expected[topic] for topic, p in publishers.items()):
                break
            time.sleep(.05)
        else:
            raise RuntimeError('Real EKF input subscriptions do not match ' + repr(expected))
        (args.output / 'effective-parameters.json').write_text(json.dumps(effective, indent=2) + '\n')
        print('OFFLINE ONLY: {} inputs, {} real EKF profiles, {:.1f}s recorded time'.format(
            len(rows), len(profiles), end-begin), flush=True)
        at, i, last_report = begin, 0, begin
        wall_start = time.monotonic()
        # Deliver each original measurement before advancing the clock to the
        # next 10 ms tick. A short wall-time yield lets roscpp drain callbacks.
        while at <= end + .1:
            for proc in processes:
                if proc.poll() is not None:
                    raise RuntimeError('EKF/master process exited; inspect logs')
            while i < len(rows) and rows[i][0] <= at:
                _, topic, msg = rows[i]
                publishers[topic].publish(msg)
                i += 1
            time.sleep(.001)
            clock.publish(Clock(rospy.Time.from_sec(at)))
            at += .01
            time.sleep(max(.001, .01 / args.rate - .001))
            if at - last_report >= 60:
                print('simulated {:.0f}s; received {}'.format(at-begin,
                      {k: len(v) for k, v in received.items()}), flush=True)
                last_report = at
        time.sleep(.2)
        for name, data in received.items():
            if not data:
                raise RuntimeError('No output from ' + name)
            (args.output / (name + '.json')).write_text(json.dumps(data) + '\n')
        result = dict(OFFLINE_ONLY=True, input_bag=str(args.bag),
                      input_sha256=hashlib.sha256(args.bag.read_bytes()).hexdigest(),
                      input_messages=len(rows), begin=begin, end=end,
                      input_topic_counts=input_counts,
                      expected_sensor_subscribers=expected,
                      imu_replay=dict(enabled=use_imu, angular_variance_rad2_s2=args.imu_variance,
                                      raw_angular_covariances=sorted(raw_covariances),
                                      source_frames=sorted(source_imu_frames),
                                      replay_frames=sorted(imu_frames),
                                      frame_name_override=args.imu_frame,
                                      frame_name_override_basis=('Explicit caller-selected offline name correction: driver reports livox_frame; configured body is the same MID360 IMU frame. Recorded body TF is used unchanged; no new extrinsic calibration or gyro bias subtraction.' if args.imu_frame else None),
                                      static_transform_count=len(static_transforms),
                                      static_tf_source='Recorded /tf_static geometry only; no dynamic TF replay or invented transforms',
                                      correlation='LIO already uses MID360 IMU; fused measurements are correlated'),
                      simulated_clock_step_s=.01, rate=args.rate,
                      elapsed_wall_s=time.monotonic()-wall_start,
                      output_counts={k: len(v) for k, v in received.items()},
                      output_columns=['stamp', 'x', 'y', 'yaw', 'vx', 'vy', 'wz', 'P_yaw', 'P_wz'],
                      package=subprocess.check_output(['dpkg-query','-W',
                                                       'ros-noetic-robot-localization'], text=True).strip(),
                      boundary='Recorded inputs; real EKF; no navigation feedback or independent pose ground truth')
        (args.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2), flush=True)
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for proc in reversed(processes):
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        for log in logs:
            log.close()


if __name__ == '__main__':
    main()
