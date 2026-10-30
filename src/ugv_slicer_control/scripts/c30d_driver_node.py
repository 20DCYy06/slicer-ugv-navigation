#!/usr/bin/env python3
"""Controlled SLC2 owner: explicit PWM, wheel-speed or MCU body/IMU control.

The independent launch is opt-in. No launch in the navigation stack starts it.
MCU ack means accepted command; 0x90 PWM is the software duty reference in
timer ticks, not CCR readback, permille or proof of bridge current, wheel
movement, braking or ground stop.
"""
import hashlib
import json
import math
from pathlib import Path
import sys
import threading
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rospy
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry as OdomMsg
from std_msgs.msg import Bool

import c30d_link
import chassis_core
import control_core
import serial_transport
import slicer_manual
from c30d_link import Chassis
from chassis_core import Odometry
from control_core import load_config, wheel_pwm, wheel_speed_mm_s, limit_pwm, Slew


def main():
    rospy.init_node('slicer_c30d_driver')
    # A mandatory path prevents a stale/missing script-relative default from
    # silently selecting the teammate configuration or another overlay.
    config_file = Path(rospy.get_param('~config_file')).resolve()
    c = load_config(config_file, rospy.get_param('~control_mode', ''))
    mode = c.get('control_mode', 'pwm_reference')
    body_mode = mode == 'body_speed'
    closed_loop = mode in ('wheel_speed', 'body_speed')
    pwm_limit = c['ros_driver_pwm_limit_permille']
    mock = rospy.get_param('~mock', False)
    publish_tf = rospy.get_param('~publish_tf', c['publish_tf'])
    observe_motor_registers = rospy.get_param('~observe_motor_registers', False)
    diagnostic_pub = rospy.Publisher('~diagnostics', DiagnosticArray, queue_size=100)
    driver_instance_id = uuid.uuid4().hex
    event_seq = 0
    event_lock = threading.Lock()

    def emit(event, **fields):
        nonlocal event_seq
        try:
            # Allocate before publication: a later gap exposes missing evidence.
            # The lock covers only numbering, never transport or control work.
            with event_lock:
                event_seq += 1
                fields['event_seq'] = event_seq
            fields['driver_instance_id'] = driver_instance_id
            fields.setdefault('host_monotonic_s', time.monotonic())
            message = DiagnosticArray()
            message.header.stamp = rospy.Time.now()
            status = DiagnosticStatus()
            status.name = rospy.get_name() + '/' + event
            status.hardware_id = 'SLC2:' + str(c['serial_number'])
            status.message = event
            status.level = DiagnosticStatus.ERROR if event == 'fault' else DiagnosticStatus.OK
            for key, item in fields.items():
                text = item if isinstance(item, str) else json.dumps(item, sort_keys=True, allow_nan=False)
                status.values.append(KeyValue(key=key, value=text))
            message.status = [status]
            diagnostic_pub.publish(message)
        except Exception as exc:
            rospy.logwarn_throttle(2, 'Driver diagnostics unavailable: %s', str(exc))

    modules = {Path(__file__).name: dict(path=str(Path(__file__).resolve()),
                                       sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())}
    for module in (control_core, slicer_manual, c30d_link, chassis_core, serial_transport):
        path = Path(module.__file__).resolve()
        modules[path.name] = dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    provenance = dict(config_file=str(config_file), config_sha256=hashlib.sha256(config_file.read_bytes()).hexdigest(),
                      module_paths_sha256=modules,
                      pwm_mapping='continuous_open_loop_reference' if mode == 'pwm_reference' else 'not_used',
                      control_mode=mode, host_pwm_limit_enforced=mode == 'pwm_reference',
                      mcu_speed_pwm_cap_reference_permille=600 if closed_loop else None,
                      speed_pwm_cap_semantics='MCU compiled reference only; installed firmware unverified; no host PWM limit in 0x10/0x16',
                      mock=mock, observe_motor_registers=observe_motor_registers,
                      effective_config=dict(c, publish_tf=publish_tf), driver_instance_id=driver_instance_id,
                      input_topic=c['output_topic'], publish_tf=publish_tf,
                      startup_host_monotonic_s=time.monotonic(),
                      ack_semantics='accepted_command_seq; not proof of actuator execution',
                      telemetry_pwm_unit='timer ticks; firmware reference period=1600; deployed identity unverified')
    # A recorder often joins after startup. Keep identity queryable without
    # latching the continuous diagnostics stream or adding a control gate.
    try:
        rospy.set_param('~provenance', provenance)
    except Exception as exc:
        rospy.logwarn('Could not store driver provenance parameter: %s', str(exc))
    rospy.loginfo('SLC2 driver provenance: %s', json.dumps(provenance, sort_keys=True))
    emit('startup', **provenance)

    target = [0., 0., -math.inf]
    lock = threading.Lock()

    def command(msg):
        values = (msg.linear.x, msg.angular.z)
        now = time.monotonic()
        valid = all(math.isfinite(x) for x in values)
        with lock:
            target[:] = [*values, now] if valid else [0., 0., -math.inf]
        emit('command_rx', command_rx_monotonic_s=now, valid=valid,
             linear_x_m_s=values[0] if math.isfinite(values[0]) else str(values[0]),
             angular_z_rad_s=values[1] if math.isfinite(values[1]) else str(values[1]))

    sub = rospy.Subscriber(c['output_topic'], Twist, command, queue_size=1)
    pub = rospy.Publisher(c['odom_topic'], OdomMsg, queue_size=5)
    ready_pub = rospy.Publisher('~ready', Bool, queue_size=1, latch=True)

    def publish_ready(value):
        # State observation must never prevent an explicit zero or serial close.
        try:
            ready_pub.publish(value)
        except Exception as exc:
            rospy.logwarn_throttle(2, 'Driver ready status unavailable: %s', str(exc))

    publish_ready(False)
    broadcaster = tf2_ros.TransformBroadcaster() if publish_tf else None
    odom = Odometry(dict(c, port_order=list('ABCD')))
    car, previous = None, None
    slew, last = Slew(c), time.monotonic()
    previous_speed = [0]*4
    next_motor_register_request = 0.
    try:
        if not mock:
            car = Chassis(c, observe=emit)
            if closed_loop:
                car.configure(c['encoder_signs'])
                # Retain only completed ACK + fresh configured state. Decimal
                # strings preserve uint32 nonce/tick beyond XML-RPC int32.
                snapshot = dict(request_nonce=str(car.nonce), ack_nonce=str(car.configuration_ack[0]),
                                accepted=car.configuration_ack[1],
                                pwm_signs_abcd=list(car.cfg['pwm_signs']),
                                encoder_signs_abcd=list(car.cfg['encoder_signs']),
                                configured=bool(car.status['flags'] & 8), mcu_tick_us=str(car.status['tick']))
                provenance['mcu_configuration'] = snapshot
                try:
                    rospy.set_param('~provenance', provenance)
                except Exception as exc:
                    rospy.logwarn('Could not store MCU configuration provenance: %s', str(exc))
                emit('mcu_configuration', **snapshot)
                if body_mode:
                    info = car.require_body_control()
                    body = car.body_telemetry
                    extension = dict(request_nonce=str(car.nonce), ack_nonce=str(info[0]),
                                     version=info[1], who_am_i=info[2],
                                     initial_info_imu_flags=info[3], initial_info_source=info[4],
                                     imu_flags=body['imu_flags'], source=body['source'])
                    provenance['body_extension'] = extension
                    try:
                        rospy.set_param('~provenance', provenance)
                    except Exception as exc:
                        rospy.logwarn('Could not store body extension provenance: %s', str(exc))
                    emit('body_extension', **extension)
            else:
                cap, duration = car.require_pwm_test()
                if cap != c['test_pwm_limit_permille'] or duration != 0:
                    raise RuntimeError('SLC2 continuous PWM capability does not match configuration')
        if body_mode:
            rospy.logwarn('SLC2 body_speed: SI Twist via 0x16, encoder vx + MCU IMU wz; fresh 0x93 feedback required. No host PWM limit or fallback to wheel/PWM mode.')
        elif mode == 'wheel_speed':
            rospy.logwarn('SLC2 wheel_speed: target ABCD mm/s via 0x10 after 0x12 configuration; MCU PI/signs/units require physical validation. Host PWM cap is not enforced; MCU reference compiled cap=600 permille, installed firmware unverified.')
        else:
            rospy.logwarn('SLC2 continuous PWM reference; physical speed/radius uncalibrated. ROS cap=%d permille (firmware cap=%d).',
                          pwm_limit, c['test_pwm_limit_permille'])
        while not rospy.is_shutdown():
            telemetry_recent = True
            if car:
                car.poll(0)
                if time.monotonic() - car.last_rx > .25:
                    for _ in range(2):
                        car.poll(.02)
                        if time.monotonic() - car.last_rx <= .25:
                            break
                age = time.monotonic() - car.last_rx
                if age > c['telemetry_timeout_s']:
                    raise RuntimeError('MCU telemetry timeout (%.3fs): stopping outputs' % age)
                telemetry_recent = age <= .30
                if not telemetry_recent:
                    rospy.logwarn_throttle(2, 'MCU telemetry delayed %.3fs; zero motor command until fresh data arrives', age)
                s = car.status
                if s['flags'] & 4:
                    if body_mode:
                        car.body_speed(0., 0.)
                    elif mode == 'wheel_speed':
                        car.speed([0]*4)  # stop before state publication or diagnostic reply
                    publish_ready(False)
                    raise RuntimeError('MCU fault (motor output stopped; switch Motor OFF to clear before retry): ' +
                                       str(car.read_fault_diagnostic()))
                if closed_loop and (not s['flags'] & 8 or s['flags'] & (64 | 128)):
                    if body_mode:
                        car.body_speed(0., 0.)
                    publish_ready(False)
                    raise RuntimeError(mode + ' configuration lost or MCU entered direct/calibration mode; stopping without PWM fallback')
            now = time.monotonic()
            with lock:
                received = target[2]
                fresh = now - received < c['command_timeout_s']
                v, w = target[:2] if fresh else (0., 0.)
            stop_reason = 'command_timeout' if not fresh else ('zero_command' if v == 0 and w == 0 else '')
            if not telemetry_recent:
                v, w, stop_reason = 0., 0., 'telemetry_delayed'
            ready = bool(car and telemetry_recent and s['flags'] & 16 and s['flags'] & 32)
            if closed_loop and car and not (s['flags'] & 16 and s['flags'] & 32):
                v, w, stop_reason = 0., 0., 'motor_disabled_or_power_missing'
            if body_mode:
                if car and not car.body_feedback_fresh(c['body_feedback_timeout_s'], c['body_imu_timeout_ms']):
                    v, w, stop_reason, ready = 0., 0., 'body_feedback_unavailable', False
                    rospy.logwarn_throttle(2, 'Body/IMU feedback unavailable or stale; zero body target until fresh feedback arrives')
                # Guard already bounds/smooths SI Twist. The MCU body controller
                # owns wheel conversion and reversal; do not insert wheel zeros.
                output = [round(v*1000), round(w*1000)]
                fields = dict(target_body_vx_mm_s=output[0], target_body_wz_mrad_s=output[1])
            elif mode == 'wheel_speed':
                desired = wheel_speed_mm_s(v, w, c)
                output = [0]*4 if any(a*b < 0 for a, b in zip(previous_speed, desired)) else desired
                previous_speed = output
                fields = dict(target_speed_mm_s_abcd=output, desired_speed_mm_s_abcd=desired)
            else:
                desired = limit_pwm(wheel_pwm(v, w, c), pwm_limit)
                output = limit_pwm(slew.step(desired, now-last), pwm_limit)
                fields = dict(desired_pwm_permille_abcd=desired, output_pwm_permille_abcd=output)
            last = now
            sequence = None
            if car:
                publish_ready(ready)
                if body_mode:
                    sequence = car.body_speed(v, w)
                else:
                    sequence = car.speed(output) if mode == 'wheel_speed' else car.pwm(output)
            emit('command_tx', command_seq=sequence, command_rx_monotonic_s=received if math.isfinite(received) else None,
                 input_age_s=now-received if math.isfinite(received) else None, stop_reason=stop_reason,
                 linear_x_m_s=v, angular_z_rad_s=w, control_mode=mode, **fields)
            if car and observe_motor_registers and any(output):
                observed_now = time.monotonic()
                if observed_now >= next_motor_register_request:
                    # At most 5Hz; normal control send is already complete.
                    # A missing reply never inserts a wait into this loop.
                    next_motor_register_request = observed_now + .2
                    try:
                        car.request_motor_registers()
                    except Exception as exc:
                        rospy.logwarn_throttle(2, 'MCU motor register observation request failed: %s', str(exc))
            if car:
                if previous and s['tick'] != previous['tick']:
                    dt = ((s['tick'] - previous['tick']) & 0xffffffff)*1e-6
                    counts = [(((b-a+2**31) % 2**32)-2**31)*sign
                              for a, b, sign in zip(previous['raw'], s['raw'], c['encoder_signs'])]
                    x, y, yaw, linear, angular = odom.update(counts, dt)
                    msg = OdomMsg()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id, msg.child_frame_id = c['odom_frame'], c['base_frame']
                    msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
                    msg.pose.pose.orientation.z = math.sin(yaw/2)
                    msg.pose.pose.orientation.w = math.cos(yaw/2)
                    msg.twist.twist.linear.x, msg.twist.twist.angular.z = linear, angular
                    for index in (0, 7, 35):
                        msg.pose.covariance[index] = msg.twist.covariance[index] = .1
                    for index in (14, 21, 28):
                        msg.pose.covariance[index] = msg.twist.covariance[index] = 1e6
                    pub.publish(msg)
                    if broadcaster:
                        tf = TransformStamped()
                        tf.header, tf.child_frame_id = msg.header, msg.child_frame_id
                        tf.transform.translation.x, tf.transform.translation.y = x, y
                        tf.transform.rotation = msg.pose.pose.orientation
                        broadcaster.sendTransform(tf)
                previous = dict(s)
            time.sleep(1/c['publish_hz'])
    except Exception as exc:
        emit('fault', error=str(exc))
        raise
    finally:
        try:
            publish_ready(False)
        finally:
            if car:
                car.close()
        emit('shutdown', serial_closed=True)


if __name__ == '__main__':
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    except Exception as exc:
        rospy.logfatal('%s', exc)
        raise SystemExit(1)
