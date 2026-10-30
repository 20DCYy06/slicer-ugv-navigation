#!/usr/bin/env python3
"""Passive ROS1 capture: subscriptions, selected parameter reads and rosbag only.

No driver imports, serial access, publishers, services, goals or control intents.
Run in an already configured ROS shell. No ROS dependency is needed for --help.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import xmlrpc.client


def topic_map(namespace='ugv', navigation=False, perception=False):
    action = '/' + '/'.join(x for x in (namespace.strip('/'), 'move_base') if x)
    prefix = action.rsplit('/', 1)[0]
    topics = dict(odom='/odom', lio='/lio/odom', wheel='/wheel/odom', lio_raw='/Odometry',
                scan='/scan', tf='/tf', tf_static='/tf_static', diagnostics='/diagnostics',
                driver_diagnostics='/slicer_c30d_driver/diagnostics', driver_ready='/slicer_c30d_driver/ready',
                raw='/cmd_vel/nav_raw', planner_raw=prefix+'/cmd_vel/nav_raw',
                nav='/cmd_vel/nav', cmd='/cmd_vel', teleop='/teleop/cmd_vel', mission='/cmd_vel/mission',
                manual_state='/cup_nav/manual_state',
                guard_reason='/slicer_nav_arc_guard/hold_reason',
                action_goal=action+'/goal', action_status=action+'/status',
                action_result=action+'/result', action_feedback=action+'/feedback')
    if navigation:
        topics.update(current_goal=action+'/current_goal', simple_goal=prefix+'/move_base_simple/goal',
                      global_plan=action+'/GlobalPlanner/plan',
                      guard_adjustment='/slicer_nav_arc_guard/adjustment', rosout='/rosout_agg')
        for scope in ('global', 'local'):
            for stream in ('costmap', 'costmap_updates', 'footprint'):
                topics[scope+'_'+stream] = action+'/'+scope+'_costmap/'+stream
        for plugin, name in (('TebLocalPlannerROS', 'teb'), ('DWAPlannerROS', 'dwa')):
            for stream in ('global_plan', 'local_plan'):
                topics[name+'_'+stream] = action+'/'+plugin+'/'+stream
    if perception:
        topics.update(lidar='/livox/lidar', imu='/livox/imu',
                      cloud_registered='/cloud_registered', cloud_body='/cloud_registered_body',
                      cloud_filtered='/cloud_nav_filtered', scan_raw='/scan_raw', scan_front='/scan_front',
                      perception_test_scan=prefix+'/perception_test/scan')
    return topics


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def redact(value):
    """Only selected robot parameters are read; never export credential fields."""
    if isinstance(value, dict):
        return {str(k): ('[redacted]' if any(x in str(k).lower() for x in
                ('password', 'passwd', 'secret', 'token', 'credential', 'api_key', 'authorization'))
                else redact(v)) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [redact(x) for x in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class StreamStats:
    def __init__(self):
        self.count = self.repeated = self.backward = self.future = self.invalid = 0
        self.first = self.last = self.header = self.last_ros = self.progress_at = None
        self.max_gap = 0.

    def observe(self, received, ros_now, header=None):
        self.count += 1
        if self.first is None:
            self.first = received
        if self.last is not None:
            self.max_gap = max(self.max_gap, received-self.last)
        self.last, self.last_ros = received, ros_now
        if header is None:
            return
        if not finite(header) or header <= 0:
            self.invalid += 1
            return
        if self.header is not None:
            self.repeated += header == self.header
            self.backward += header < self.header
        if self.header is None or header > self.header:
            self.progress_at = received
        self.future += header > ros_now+.1
        self.header = header

    def summary(self, now, ros_now=None):
        age_now = self.last_ros if ros_now is None else ros_now
        return dict(count=self.count,
                    receive_hz=(self.count-1)/(self.last-self.first)
                    if self.count > 1 and self.last > self.first else None,
                    max_receive_gap_s=self.max_gap if self.count > 1 else None,
                    last_receive_age_s=now-self.last if self.last is not None else None,
                    last_header_stamp_s=self.header,
                    last_header_age_s=age_now-self.header if self.header is not None else None,
                    header_progress_age_s=now-self.progress_at if self.progress_at is not None else None,
                    repeated_header_count=self.repeated, backward_header_count=self.backward,
                    future_header_count=self.future, invalid_header_count=self.invalid)


def driver_mode_summary(params):
    """Report explicit driver provenance; capability/ACK evidence is not motion."""
    params = params if isinstance(params, dict) else {}
    provenance = params.get('provenance', {})
    provenance = provenance if isinstance(provenance, dict) else {}
    reported, requested = provenance.get('control_mode', 'unknown'), params.get('control_mode', 'unknown')
    configuration = provenance.get('mcu_configuration', {})
    extension = provenance.get('body_extension', {})
    configuration = configuration if isinstance(configuration, dict) else {}
    extension = extension if isinstance(extension, dict) else {}
    findings = []

    def nonce_matches(fields):
        pair = [fields.get(key) for key in ('request_nonce', 'ack_nonce')]
        if any(isinstance(item, bool) or not isinstance(item, (int, str))
               or not str(item).isdigit() for item in pair):
            return False
        return 0 <= int(pair[0]) <= 0xffffffff and int(pair[0]) == int(pair[1])

    if 'body_speed' in (reported, requested):
        if reported != 'body_speed':
            findings.append('body_speed runtime provenance not confirmed')
        if not (nonce_matches(configuration) and configuration.get('accepted') == 1
                and configuration.get('configured') is True):
            findings.append('body_speed MCU configuration not confirmed')
        flags = extension.get('imu_flags')
        if not (nonce_matches(extension) and extension.get('version') == 1
                and extension.get('who_am_i') == 0xea and type(flags) is int and flags & 7 == 7 and not flags & 8
                and type(extension.get('source')) is int):
            findings.append('body_speed extension capability not confirmed')
    elif reported != 'wheel_speed':
        findings.append('wheel_speed runtime provenance not confirmed')
    return dict(driver_reported_control_mode=reported, driver_requested_control_mode=requested,
                driver_mcu_configuration=configuration, driver_body_extension=extension, findings=findings,
                mode_note='Requested parameters, capability ACKs and zero frames do not establish physical motion or stopping; inspect explicit runtime provenance and feedback.')


def body_feedback_summary(rows, now, stale_after):
    """Summarize received 0x93 records using their own MCU clock and IMU fields."""
    stream, advancing, invalid_imu, faults = StreamStats(), [], 0, 0
    for received, fields in rows:
        stream.observe(received, received)
        tick, flags, age = fields.get('mcu_us'), fields.get('imu_flags'), fields.get('imu_age_ms')
        if type(tick) is int and 0 <= tick < 2**32 and (not advancing or
                0 < (tick-advancing[-1][1]['mcu_us']) % 2**32 < 2**31):
            advancing.append((received, fields))
        invalid_imu += not (fields.get('version') == 1 and fields.get('who_am_i') == 0xea
                            and type(flags) is int and flags & 7 == 7 and not flags & 8
                            and finite(age) and 0 <= age <= 100)
        status = fields.get('status_flags')
        faults += type(status) is int and status >> 8 != 0
    result, findings = stream.summary(now), []
    if not rows:
        findings.append('body feedback: no messages')
    elif result['last_receive_age_s'] > stale_after:
        findings.append('body feedback: receive stale')
    if len(advancing) != len(rows):
        findings.append('body feedback: repeated/backward MCU timestamps')
    if rows and not any(b[0]-a[0] <= stale_after for a, b in zip(advancing, advancing[1:])):
        findings.append('body feedback: MCU timestamp progression not confirmed')
    if invalid_imu:
        findings.append('body feedback: invalid/stale IMU observations')
    if faults:
        findings.append('body feedback: MCU fault flags observed')
    result.update(advancing_mcu_frame_count=len(advancing),
                  repeated_or_backward_mcu_tick_count=len(rows)-len(advancing),
                  invalid_imu_feedback_count=invalid_imu, fault_feedback_count=faults,
                  last_values=rows[-1][1] if rows else None,
                  sources_observed=sorted({fields['source'] for _, fields in rows if type(fields.get('source')) is int}),
                  findings=findings)
    return result


def record_command(directory, topics, duration):
    return ['rosbag', 'record', '--lz4', '-b', '32', '--duration='+str(duration),
            '-O', str(Path(directory)/'motion.bag'),
            '__name:=ugv_readonly_bag_'+str(os.getpid())] + list(topics)


class ReadTimeout(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 3
        return connection


def rpc(master, method, *args):
    code, message, value = getattr(master, method)('/ugv_readonly_capture', *args)
    if code != 1:
        raise RuntimeError(message)
    return value


def snapshot(master, namespace, perception=False):
    result = {'captured_wall_utc': datetime.now(timezone.utc).isoformat(), 'parameters': {}}
    result['system_state'] = dict(zip(('publishers', 'subscribers', 'services'), rpc(master, 'getSystemState')))
    result['topic_types'] = dict(rpc(master, 'getTopicTypes'))
    prefix = '/' + namespace.strip('/') if namespace.strip('/') else ''
    paths = ('/use_sim_time', '/slicer_ekf', '/slicer_c30d_driver', '/slicer_nav_arc_guard',
             '/slicer_velocity_mux', prefix+'/move_base')
    if perception:
        paths += ('/slicer_cloud_to_scan', '/slicer_arm_cloud_filter', '/slicer_scan_filter',
                  '/slicer_geometry', prefix+'/perception_test/cloud_to_scan',
                  '/preprocess', '/publish', '/filter_size_surf',
                  '/filter_size_map', '/point_filter_num', '/frame_id', '/user_config_path',
                  '/common', '/mapping')
    for path in paths:
        try:
            result['parameters'][path] = redact(rpc(master, 'getParam', path))
        except Exception as exc:
            result['parameters'][path] = {'unavailable': str(exc)}
    return result


def stop_recorder(child):
    if child is None:
        return None
    if child.poll() is None:
        try:
            os.killpg(child.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    try:
        return child.wait(timeout=10)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(child.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            return child.wait(timeout=3)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            return child.wait(timeout=3)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60., help='wall seconds; Ctrl-C finalizes this recorder only')
    parser.add_argument('--output', type=Path, default=None, help='new output directory; existing files are not replaced')
    parser.add_argument('--namespace', default='ugv')
    parser.add_argument('--lio-topic', default='/lio/odom')
    parser.add_argument('--navigation', action='store_true',
                        help='also record costmaps, footprints, global/TEB/DWA plans, goals and Guard adjustments')
    parser.add_argument('--perception', action='store_true',
                        help='also record raw/intermediate point clouds, IMU and scan stages; use a short static capture (large data)')
    parser.add_argument('--extra-topic', action='append', default=[], help='explicit additional feedback topic, never all topics')
    parser.add_argument('--stale-after', type=float, default=.5, help='reporting threshold only; no control effect')
    args = parser.parse_args(argv)
    if not finite(args.duration) or args.duration <= 0 or not finite(args.stale_after) or args.stale_after <= 0:
        parser.error('duration and stale-after must be finite and positive')
    try:
        import rospy
        import roslib.message
        import roslib.packages
        import tf2_ros
    except ImportError as exc:
        parser.exit(2, 'Run in a ROS1 Noetic shell: '+str(exc)+'\n')
    master = xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'),
                                      transport=ReadTimeout(), allow_none=True)
    try:
        before = snapshot(master, args.namespace, args.perception)
    except Exception as exc:
        parser.exit(2, 'ROS master unavailable; nothing started: '+str(exc)+'\n')
    mapping = topic_map(args.namespace, args.navigation, args.perception)
    mapping['lio'] = args.lio_topic
    mux = before['parameters'].get('/slicer_velocity_mux', {})
    mapping['manual_state'] = mux.get('manual_state_topic', mapping['manual_state'])
    forbidden = {'/cup_nav/manual_intent', mux.get('manual_intent_topic', '/cup_nav/manual_intent')}
    topics = sorted(set(mapping.values()) | set(args.extra_topic))
    if forbidden.intersection(topics):
        parser.error('Do not subscribe to control intents: older control clients require the unique Mux subscriber')
    directory = args.output or Path('evidence')/datetime.now().strftime('odom-%Y%m%d-%H%M%S')
    directory.mkdir(parents=True, exist_ok=False)
    report = dict(read_only=True, data_origin='ROS observations; physical motion is not established',
                  topic_map=mapping, selected_topics=topics, before=before, findings=[],
                  clock_notes=dict(rates='collector monotonic receive time',
                                   header_age='ROS now minus message header, both ROS clock',
                                   bag='use rosbag record timestamps for manual analysis windows',
                                   mcu='MCU ticks are a separate clock; do not subtract host times'))
    template = dict(real_displacement_m=None, real_forward_m=None, real_lateral_m=None,
                    real_stop_distance_m=None, window=dict(start_s=None, end_s=None),
                    stop=dict(command_zero_s=None, settled_s=None),
                    note='Manual measurements only. Times use bag_record_s; null means unknown. Use one fixed body marker.')
    (directory/'measurements.template.json').write_text(json.dumps(template, indent=2)+'\n')
    (directory/'manifest.json').write_text(json.dumps(report, indent=2)+'\n')
    rospy.init_node('ugv_readonly_capture', anonymous=True, disable_signals=True, disable_rosout=True)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    lock, stats, tf_stats, owners, last, protocol = threading.Lock(), {}, {}, {}, {}, Counter()
    frames, decode_errors, instances = {}, Counter(), set()
    body_feedback, driver_events = [], Counter()

    def observe(raw, topic):
        received, ros_now = time.monotonic(), rospy.Time.now().to_sec()
        connection = raw._connection_header or {}
        kind = connection.get('type', '')
        try:
            # rospy shares a topic transport with tf2's typed subscriber; its
            # callback can therefore deliver an already decoded TFMessage.
            msg = raw
            if hasattr(raw, '_buff'):
                cls = roslib.message.get_message_class(kind)
                if cls is None:
                    raise ValueError('message class unavailable: '+kind)
                msg = cls().deserialize(raw._buff)
            header = getattr(msg, 'header', None)
            stamp = header.stamp.to_sec() if header else None
            with lock:
                stats.setdefault(topic, StreamStats()).observe(received, ros_now, stamp)
                owners.setdefault(topic, set()).add(connection.get('callerid', 'unknown'))
                if header:
                    frames[topic] = header.frame_id
                if kind == 'tf2_msgs/TFMessage':
                    for transform in msg.transforms:
                        edge = transform.header.frame_id+' -> '+transform.child_frame_id
                        static = topic == '/tf_static'
                        key = ('static: ' if static else '')+edge
                        tf_stats.setdefault(key, StreamStats()).observe(received, ros_now,
                            None if static else transform.header.stamp.to_sec())
                        owners.setdefault(key, set()).add(connection.get('callerid', 'unknown'))
                if kind == 'geometry_msgs/Twist':
                    last[topic] = {'linear_x': msg.linear.x, 'angular_z': msg.angular.z}
                elif kind in ('std_msgs/Bool', 'std_msgs/String'):
                    value = msg.data
                    if kind.endswith('String'):
                        try:
                            value = json.loads(value)
                        except (ValueError, TypeError):
                            pass
                    last[topic] = value
                if kind == 'diagnostic_msgs/DiagnosticArray':
                    for status in msg.status:
                        fields = {}
                        for item in status.values:
                            try:
                                fields[item.key] = json.loads(item.value)
                            except (ValueError, TypeError):
                                fields[item.key] = item.value
                        driver_events[status.message] += 1
                        if status.message == 'serial_tx':
                            protocol[str(fields.get('kind', 'unknown'))] += 1
                        elif status.message == 'body_telemetry':
                            body_feedback.append((received, fields))
                        for field in status.values:
                            if field.key == 'driver_instance_id':
                                instances.add(field.value)
        except Exception as exc:
            with lock:
                decode_errors[str(exc)] += 1

    subscribers = [rospy.Subscriber(topic, rospy.AnyMsg, observe, callback_args=topic, queue_size=100) for topic in topics]
    child, interrupted, recorder_log = None, False, None
    began = time.monotonic()
    print('CAPTURE WINDOW: %.1f s from collector start; stops automatically, including while waiting. '
          'Ctrl-C finalizes this recorder only. Output: %s' % (args.duration, directory), flush=True)
    if args.perception:
        print('PERCEPTION CAPTURE: raw/intermediate point clouds enabled; large data, use a short static window.',
              flush=True)
    try:
        recorder_log = (directory/'rosbag.log').open('w')
        command = record_command(directory.resolve(), topics, args.duration)
        report['record_command'] = command
        child = subprocess.Popen(command, stdout=recorder_log, stderr=subprocess.STDOUT, start_new_session=True)
        # A recorder process alone is not evidence that topic subscriptions exist.
        available = {topic for topic, nodes in before['system_state']['publishers'] if nodes}.intersection(topics)
        while time.monotonic()-began < min(4., args.duration):
            if child.poll() is not None:
                raise RuntimeError('rosbag exited before recording; inspect rosbag.log')
            state = dict(zip(('publishers', 'subscribers', 'services'), rpc(master, 'getSystemState')))
            registered = {topic for topic, nodes in state['subscribers'] if any(
                node.rsplit('/', 1)[-1] == 'ugv_readonly_bag_'+str(os.getpid()) for node in nodes)}
            if available and available <= registered:
                report['recorder_subscriptions_confirmed'] = True
                break
            time.sleep(.1)
        if report.get('recorder_subscriptions_confirmed'):
            print('RECORDING STARTED: '+str(directory)+' (read-only; no motion authorization)', flush=True)
        else:
            report['findings'].append('recorder_subscriptions_not_confirmed')
            print('RECORDING NOT CONFIRMED: inspect manifest/log; do not begin a motion trial', flush=True)
        while time.monotonic()-began < args.duration:
            if child.poll() is not None:
                break
            time.sleep(.05)
    except KeyboardInterrupt:
        interrupted = True
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        report['recorder_exit_code'] = stop_recorder(child)
        if recorder_log:
            recorder_log.close()
        now, ros_now = time.monotonic(), rospy.Time.now().to_sec()
        with lock:
            report['topics'] = {topic: stats.get(topic, StreamStats()).summary(now, ros_now) for topic in topics}
            report['tf_edges'] = {edge: value.summary(now, ros_now) for edge, value in tf_stats.items()}
            report['observed_publishers'] = {key: sorted(value) for key, value in owners.items()}
            report['last_values'], report['frame_ids'] = last, frames
            report['decode_errors'], report['observed_tx_kind_counts'] = dict(decode_errors), dict(protocol)
            report['observed_driver_instances'] = sorted(instances)
            report['observed_driver_event_counts'] = dict(driver_events)
            report['observed_body_feedback'] = body_feedback_summary(body_feedback, now, args.stale_after)
        report['wall_duration_s'], report['interrupted'] = now-began, interrupted
        try:
            report['after'] = snapshot(master, args.namespace, args.perception)
        except Exception as exc:
            report['after_error'] = str(exc)
        report['tf_lookup'] = {}
        for target, source in [('odom', 'base_link'), ('base_link', frames.get(mapping['scan'], 'base_link'))]:
            try:
                transform = buffer.lookup_transform(target, source, rospy.Time(0))
                report['tf_lookup'][target+' <- '+source] = dict(
                    available=True, stamp_s=transform.header.stamp.to_sec(),
                    same_frame=target == source,
                    translation=[getattr(transform.transform.translation, key) for key in ('x', 'y', 'z')])
            except Exception as exc:
                report['tf_lookup'][target+' <- '+source] = dict(available=False, error=str(exc))
        report['observer_package_resolution'] = {}
        for package in ('ugv_slicer_control', 'ugv_bringup', 'slicer_bringup'):
            try:
                value = subprocess.run(['rospack', 'find', package], text=True, capture_output=True, timeout=3)
                report['observer_package_resolution'][package] = value.stdout.strip() or value.stderr.strip()
            except Exception as exc:
                report['observer_package_resolution'][package] = str(exc)
        params = report.get('after', before)['parameters'].get('/slicer_c30d_driver', {})
        mode = driver_mode_summary(params)
        report['findings'].extend(mode.pop('findings'))
        report.update(mode)
        if 'body_speed' in (report['driver_reported_control_mode'], report['driver_requested_control_mode']):
            report['findings'].extend(report['observed_body_feedback']['findings'])
        if report['observed_driver_event_counts'].get('fault'):
            report['findings'].append('driver fault diagnostics observed')
        for key in ('odom', 'lio', 'wheel', 'scan'):
            summary = report['topics'][mapping[key]]
            if not summary['count']:
                report['findings'].append(key+': no messages')
            elif summary['last_receive_age_s'] > args.stale_after:
                report['findings'].append(key+': receive stale')
            if summary['last_header_age_s'] is not None and abs(summary['last_header_age_s']) > args.stale_after:
                report['findings'].append(key+': header age outside observation threshold')
            if summary['repeated_header_count'] or summary['backward_header_count']:
                report['findings'].append(key+': repeated/backward source timestamps')
            if summary['invalid_header_count'] or summary['future_header_count']:
                report['findings'].append(key+': invalid/future source timestamps')
        if not report['tf_lookup']['odom <- base_link']['available']:
            report['findings'].append('odom <- base_link TF unavailable')
        if len(report['observed_publishers'].get('odom -> base_link', [])) > 1:
            report['findings'].append('multiple odom -> base_link TF publishers')
        if len(report['observed_driver_instances']) > 1:
            report['findings'].append('multiple driver instances observed; do not combine their MCU sequences')
        report['bag_finalized'] = (directory/'motion.bag').is_file() and not (directory/'motion.bag.active').exists()
        if not report['bag_finalized']:
            report['findings'].append('bag not finalized; preserve .active and recorder log')
        (directory/'manifest.json').write_text(json.dumps(redact(report), indent=2, allow_nan=False)+'\n')
        for subscription in subscribers:
            subscription.unregister()
        rospy.signal_shutdown('passive capture complete')
    print('CAPTURE COMPLETE: recorder stopped; no further messages are recorded. Output: '+str(directory)+
          '; mode='+report['driver_reported_control_mode']+
          ', bag_finalized='+str(report['bag_finalized']), flush=True)
    for key in ('lio', 'odom', 'wheel', 'scan'):
        value = report['topics'][mapping[key]]
        print('%s: n=%d Hz=%s receive_age=%s header_age=%s' % (
            key, value['count'], value['receive_hz'], value['last_receive_age_s'], value['last_header_age_s']))
    for finding in report['findings']:
        print('CHECK: '+finding)
    return 1 if report.get('error') or not report['bag_finalized'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
