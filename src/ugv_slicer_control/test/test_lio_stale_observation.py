#!/usr/bin/env python3
"""TEST_ONLY: run the real Guard against synthetic, isolated ROS inputs.

Run only in a --network none, no-device container. Starts its own loopback
master; never starts a driver/Mux and never uses production velocity topics.
The unchanged production Guard is expected to FAIL the LIO safety checks.
"""
import argparse
from collections import deque
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--guard', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'scripts/arc_velocity_guard.py')
    args = parser.parse_args()
    if os.environ.get('UGV_OFFLINE_TEST') != '1' or not Path('/.dockerenv').exists():
        parser.error('requires the explicit network-none, no-device Docker test command')
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:12415', ROS_IP='127.0.0.1')
    os.environ.pop('ROS_HOSTNAME', None)
    log_files, children = [], []
    result = dict(test_only=True, motion_test=False, driver_or_mux_started=False,
                  guard_path=str(args.guard),
                  guard_sha256=hashlib.sha256(args.guard.read_bytes()).hexdigest(),
                  lio_timeout_s=.5, cases=[], passed=False)
    fixture = None
    try:
        master_log = (args.output / 'master.log').open('w')
        log_files.append(master_log)
        children.append(subprocess.Popen(['roscore', '-p', '12415'], stdout=master_log,
                                         stderr=subprocess.STDOUT, start_new_session=True))
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            try:
                if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/TEST_ONLY_probe')[0] == 1:
                    break
            except (OSError, xmlrpc.client.Error):
                time.sleep(.1)
        else:
            raise RuntimeError('isolated master did not start')
        import rospy
        import tf
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import OccupancyGrid, Odometry
        from std_msgs.msg import String

        rospy.init_node('TEST_ONLY_lio_probe', disable_signals=True)
        namespace = '/TEST_ONLY_lio_guard'
        planner = '/TEST_ONLY_lio_planner'
        root = Path(__file__).resolve().parents[2]
        rospy.set_param(planner + '/base_local_planner', 'dwa_local_planner/DWAPlannerROS')
        rospy.set_param(planner + '/DWAPlannerROS', dict(
            max_vel_x=.125, min_vel_x=0., max_vel_trans=.125, max_vel_theta=.25,
            acc_lim_x=.2, acc_lim_trans=.2, acc_lim_theta=1.))
        rospy.set_param(planner + '/local_costmap/robot_base_frame', 'TEST_ONLY_lio_base')
        rospy.set_param(planner + '/local_costmap/footprint',
                        [[-.165, -.14], [-.165, .14], [.27, .14], [.27, -.14]])
        rospy.set_param(namespace, dict(
            config_file=str(root / 'ugv_bringup/config/test_slicer_guard.yaml'),
            control_config_file=str(root / 'ugv_bringup/config/test_slicer_guard.yaml'),
            planner_namespace=planner, input_topic='/TEST_ONLY_lio/raw',
            output_topic='/TEST_ONLY_lio/output', costmap_topic='/TEST_ONLY_lio/grid',
            lio_topic='/TEST_ONLY_lio/lio_odom', lio_timeout=.5,
            command_timeout=.5, publish_hz=20., costmap_timeout=1.5, tf_timeout=.5,
            compensation_horizon=.8, turning_linear_window=.03,
            angular_intent_threshold=.01, turn_compensation_enabled=False,
            allow_patrol_spin=False))

        class Fixture:
            def __init__(self):
                self.lock = threading.RLock()
                self.stop = threading.Event()
                self.outputs = deque(maxlen=1000)
                self.reason = None
                self.lio_mode = 'fresh'
                self.enabled = dict(command=True, grid=True, tf=True)
                self.sent = dict.fromkeys(('command', 'grid', 'tf', 'lio', 'odom', 'wheel'), 0)
                self.last_sent = dict.fromkeys(self.sent, None)
                self.observed = dict.fromkeys(('lio', 'odom', 'wheel'), 0)
                self.frozen_stamp = rospy.Time.now()
                self.broadcaster = tf.TransformBroadcaster()
                self.publishers = {kind: rospy.Publisher('/TEST_ONLY_lio/' + topic, cls, queue_size=1)
                                   for kind, topic, cls in (
                                       ('command', 'raw', Twist), ('grid', 'grid', OccupancyGrid),
                                       ('lio', 'lio_odom', Odometry), ('odom', 'ekf_odom', Odometry),
                                       ('wheel', 'wheel_odom', Odometry))}
                rospy.Subscriber('/TEST_ONLY_lio/output', Twist, self.output, queue_size=100)
                rospy.Subscriber(namespace + '/hold_reason', String, self.hold, queue_size=10)
                for kind in self.observed:
                    topic = {'lio': 'lio_odom', 'odom': 'ekf_odom', 'wheel': 'wheel_odom'}[kind]
                    rospy.Subscriber('/TEST_ONLY_lio/' + topic, Odometry, self.observe,
                                     callback_args=kind, queue_size=30)
                self.thread = threading.Thread(target=self.publish, daemon=True)
                self.thread.start()

            def output(self, msg):
                with self.lock:
                    self.outputs.append((time.monotonic(), msg.linear.x, msg.angular.z))

            def hold(self, msg):
                with self.lock:
                    self.reason = msg.data

            def observe(self, msg, kind):
                with self.lock:
                    self.observed[kind] += 1

            def publish(self):
                while not self.stop.is_set() and not rospy.is_shutdown():
                    with self.lock:
                        now, stamp = time.monotonic(), rospy.Time.now()
                        if self.enabled['tf']:
                            self.broadcaster.sendTransform((0., 0., 0.), (0., 0., 0., 1.), stamp,
                                                           'TEST_ONLY_lio_base', 'TEST_ONLY_lio_grid')
                            self.sent['tf'] += 1
                            self.last_sent['tf'] = now
                        for kind in ('command', 'grid', 'lio', 'odom', 'wheel'):
                            if kind in self.enabled and not self.enabled[kind]:
                                continue
                            if kind == 'lio' and self.lio_mode == 'stopped':
                                continue
                            if kind == 'command':
                                msg = Twist(); msg.linear.x = .06; msg.angular.z = .02
                            elif kind == 'grid':
                                msg = OccupancyGrid(); msg.header.frame_id = 'TEST_ONLY_lio_grid'
                                msg.header.stamp = stamp; msg.info.resolution = .05
                                msg.info.width = msg.info.height = 80
                                msg.info.origin.position.x = msg.info.origin.position.y = -2.
                                msg.info.origin.orientation.w = 1.; msg.data = [0] * 6400
                            else:
                                msg = Odometry(); msg.header.frame_id = 'TEST_ONLY_lio_grid'
                                msg.child_frame_id = 'TEST_ONLY_lio_base'
                                msg.header.stamp = (self.frozen_stamp if kind == 'lio' and self.lio_mode == 'frozen'
                                                    else stamp - rospy.Duration(5.) if kind == 'lio' and self.lio_mode == 'old'
                                                    else stamp)
                                msg.pose.pose.orientation.w = 1.; msg.twist.twist.linear.x = .06
                            self.publishers[kind].publish(msg)
                            self.sent[kind] += 1; self.last_sent[kind] = now
                    self.stop.wait(.03)

            def moving(self):
                with self.lock:
                    self.lio_mode = 'fresh'
                    self.enabled = dict(command=True, grid=True, tf=True)
                    start = time.monotonic()
                deadline = start + 5.
                while time.monotonic() < deadline:
                    with self.lock:
                        rows = [x for x in self.outputs if x[0] >= start][-4:]
                        ready = (len(rows) == 4 and self.reason == '' and
                                 all(abs(x[1] - .06) < .002 and abs(x[2] - .02) < .002 for x in rows))
                    if ready:
                        return
                    time.sleep(.02)
                raise RuntimeError('fixture never reached steady permitted Guard output: ' + str(self.reason))

            def phase(self, name, duration, lio_mode='fresh', disable=None):
                self.moving()
                with self.lock:
                    start = time.monotonic(); before = dict(self.sent); received = dict(self.observed)
                    self.frozen_stamp = rospy.Time.now(); self.lio_mode = lio_mode
                    if disable:
                        self.enabled[disable] = False
                time.sleep(duration)
                with self.lock:
                    end = time.monotonic()
                    tail = [x for x in self.outputs if x[0] >= start + duration - .3]
                    passed = len(tail) >= 4 and all(x[1:] == (0., 0.) for x in tail)
                    row = dict(name=name, desired_zero=True, passed=passed, duration_s=duration,
                               last_hold_reason=self.reason, tail_output_count=len(tail),
                               tail_nonzero_count=sum(x[1:] != (0., 0.) for x in tail),
                               last_output=tail[-1][1:] if tail else None,
                               sent_during_phase={k:self.sent[k]-before[k] for k in before},
                               received_odometry={k:self.observed[k]-received[k] for k in received},
                               source_last_send_age_s={k:end-v if v is not None else None
                                                       for k,v in self.last_sent.items()},
                               output_samples=[[x[0]-start,x[1],x[2]] for x in self.outputs if x[0]>=start])
                    fresh = set(self.sent) - ({disable} if disable else set())
                    if lio_mode == 'stopped':
                        fresh.remove('lio')
                    row['other_inputs_fresh'] = all(
                        row['sent_during_phase'][kind] >= 5 and
                        row['source_last_send_age_s'][kind] < .2 for kind in fresh)
                    row['other_inputs_fresh'] &= all(
                        row['received_odometry'][kind] >= 5
                        for kind in ('odom', 'wheel') + (() if lio_mode == 'stopped' else ('lio',)))
                    if lio_mode == 'stopped':
                        row['other_inputs_fresh'] &= row['received_odometry']['lio'] == 0
                    row['passed'] &= row['other_inputs_fresh']
                return row

        fixture = Fixture()
        guard_log = (args.output / 'guard.log').open('w'); log_files.append(guard_log)
        children.append(subprocess.Popen([sys.executable, str(args.guard), '__name:=TEST_ONLY_lio_guard'],
                                         stdout=guard_log, stderr=subprocess.STDOUT, start_new_session=True))
        for name, duration, mode, disabled in (
            ('lio_stopped_others_fresh',1.2,'stopped',None),
            ('lio_frozen_header_others_fresh',1.2,'frozen',None),
            ('lio_old_advancing_header_others_fresh',1.2,'old',None),
            ('command_stopped_lio_and_geometry_fresh',.9,'fresh','command'),
            ('tf_stopped_lio_and_inputs_fresh',.9,'fresh','tf'),
            ('grid_stopped_lio_and_tf_fresh',1.9,'fresh','grid')):
            result['cases'].append(fixture.phase(name,duration,mode,disabled))
        result['passed'] = all(x['passed'] for x in result['cases'])
    except BaseException as error:
        result['error'] = repr(error)
    finally:
        if fixture is not None:
            fixture.stop.set(); fixture.thread.join(2.)
        for process in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM); process.wait(timeout=5)
        for stream in log_files:
            stream.close()
        (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:result[k] for k in ('test_only','guard_sha256','passed')},indent=2))
        for row in result['cases']:
            print(row['name'], row['passed'], row['last_output'], row['last_hold_reason'])
        if 'error' in result:
            print(result['error'],file=sys.stderr)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
