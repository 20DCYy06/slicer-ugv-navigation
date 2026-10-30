#!/usr/bin/env python3
"""TEST_ONLY: actual driver and SLC2 codec, anonymous pipes instead of devices.

ROS messages/transport and clocks are replaced; callback, mapping, slew,
Chassis handshake, PWM framing and telemetry parser are the production code.
"""
import contextlib
import copy
import importlib.util
import json
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import xmlrpc.client

import yaml

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
CONFIG = dict(serial_port='auto', usb_vid=0x1a86, usb_pid=0x55d4,
              serial_number='TEST_ONLY', baud=115200, counts_per_revolution=60000,
              max_wheel_mm_s=600, test_pwm_limit_permille=750,
              ros_driver_pwm_limit_permille=600, test_pwm_signs=[1, -1, 1, -1],
              encoder_signs=[-1, 1, -1, 1],
              wheel_ports=dict(left_front='D', right_front='C', left_rear='B', right_rear='A'),
              wheel_diameter=.085, track_width=.15, test_duty_permille=250,
              wasd_spin_pwm_permille=300, test_slew_permille_s=1000,
              turn_gain_permille_per_rad_s=283.11258278, linear_speed=.22,
              initial_scale=1., min_scale=.2, max_scale=2., scale_step=.2,
              key_timeout_s=.25, telemetry_timeout_s=.5, publish_hz=20,
              command_timeout_s=.25, teleop_topic='/audit/teleop', nav_topic='/audit/nav',
              mission_topic='/audit/mission', output_topic='/audit/final',
              odom_topic='/audit/wheel', odom_frame='odom', base_frame='base_link',
              publish_tf=False)
REGISTER_VALUES = (1, 129, 4369, 32768, 1111, 222, 333, 444,
                   555, 666, 777, 888, 8, 819, 51)


def twist(v=0., w=0.):
    return NS(linear=NS(x=v, y=0., z=0.), angular=NS(x=0., y=0., z=w))


class Odom:
    def __init__(self):
        self.header = NS(stamp=None, frame_id='')
        self.child_frame_id = ''
        self.pose = NS(pose=NS(position=NS(x=0., y=0., z=0.),
                               orientation=NS(x=0., y=0., z=0., w=1.)), covariance=[0.]*36)
        self.twist = NS(twist=twist(), covariance=[0.]*36)


class DiagnosticArray:
    def __init__(self):
        self.header = NS(stamp=None)
        self.status = []


class DiagnosticStatus:
    OK, WARN, ERROR = 0, 1, 2

    def __init__(self, **kwargs):
        self.level, self.name, self.message, self.hardware_id = 0, '', '', ''
        self.values = []
        self.__dict__.update(kwargs)


def value(text):
    try:
        return json.loads(text)
    except ValueError:
        return text


class DriverTests(unittest.TestCase):
    def load_driver(self):
        self.assertTrue((SCRIPTS / 'c30d_driver_node.py').is_file(),
                        'controlled driver must be implemented in the new package')
        original_path = list(sys.path)
        sys.path.insert(0, str(SCRIPTS))
        self.addCleanup(lambda: sys.path.__setitem__(slice(None), original_path))
        spec = importlib.util.spec_from_file_location('driver_under_test', SCRIPTS / 'c30d_driver_node.py')
        driver = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, self.message_modules()):
            spec.loader.exec_module(driver)
        self.assertEqual(Path(driver.wheel_pwm.__code__.co_filename).resolve(),
                         SCRIPTS / 'control_core.py')
        return driver

    @staticmethod
    def message_modules():
        return {'rospy': NS(), 'tf2_ros': NS(),
                'geometry_msgs.msg': NS(Twist=twist, TransformStamped=object),
                'nav_msgs.msg': NS(Odometry=Odom), 'std_msgs.msg': NS(Bool=lambda x=False: x),
                'diagnostic_msgs.msg': NS(DiagnosticArray=DiagnosticArray,
                                          DiagnosticStatus=DiagnosticStatus, KeyValue=NS)}

    def run_driver(self, commands, telemetry=None, config=None, interval=.05,
                   flags=None, fail_pwm=False, diagnostics_fail=False, omit_config=False,
                   burst_telemetry=None, ros_params=None, config_after_start=None,
                   concurrent_commands=0, diagnostics_fail_at=None,
                   configure_ack=True, configure_nonce_matches=True,
                   configured_after_config=True, fail_speed=False, fresh_config_status=True,
                   handshake_flags=None, ready_fail_at=None, set_param_fail_at=None,
                   diagnostics_subscribe_after_start=False, configure_tick_us=None,
                   motor_register_reply=True, fail_motor_register_request=False,
                   register_frames_each_loop=False, body_capability=True,
                   body_nonce_matches=True, body_version=1, body_who=0xea,
                   body_startup_imu_flags=7, body_telemetry=None,
                   body_imu_flags=None, body_imu_age_ms=None, body_ticks=None,
                   body_frame_seq=None, fail_body=False, body_startup_feedback=True,
                   body_startup_tick_us=None):
        driver = self.load_driver()
        link_module = driver.c30d_link
        core = driver.chassis_core
        clock, callbacks, publishers, links = [10.], {}, {}, []
        params, info_logs, warn_logs = {}, [], []
        index, telemetry_count = [-1], [0]
        diagnostic_attempts, ready_attempts = [0], [0]
        set_param_attempts = [0]
        diagnostics_subscribed = [not diagnostics_subscribe_after_start]
        cfg = dict(CONFIG, **(config or {}))

        class FakeLink:
            def __init__(inner, device, expected, restore_on_close=True):
                if device != '/TEST_ONLY_SERIAL' or expected != ('1a86', '55d4', 'TEST_ONLY'):
                    raise AssertionError('device identity boundary changed')
                inner.fd, inner.writer = os.pipe()
                inner.sent, inner.closed, inner.seq = [], False, 0
                inner.ack, inner.pwm, inner.target_speed = 0, [0]*4, [0]*4
                inner.target_body, inner.body_started = [0, 0], False
                inner.configured = False
                inner.parser = core.Parser()
                links.append(inner)

            def emit(inner, kind, data):
                inner.seq = (inner.seq + 1) & 65535
                os.write(inner.writer, core.packet(kind, inner.seq, data))

            def status(inner, current_flags=None, tick=None):
                if current_flags is None:
                    current_flags = 48 | (8 if inner.configured else 0)
                telemetry_count[0] += 1
                fields = (int(clock[0]*1e6) & 0xffffffff if tick is None else tick,
                          50000, 0, 0, 0, 0, current_flags, inner.ack,
                          0, 0, 0, 0, 0, 0, 0, 0,
                          *(int(p*1.6) for p in inner.pwm), 12500)
                inner.emit(0x90, struct.pack('<II4iBH4i4h4hH', *fields))

            def body_status(inner, imu_flags=7, imu_age_ms=0, tick=None, frame_seq=None):
                # v1 fixed 64-byte teammate format. Encoder observations stay zero;
                # body/IMU estimates intentionally differ from wheel odometry.
                fields = (1, imu_flags, body_who, 1,
                          int(clock[0]*1e6) & 0xffffffff if tick is None else tick,
                          *([0]*20), 11, 22, *inner.target_body,
                          12500, 56 if inner.configured else 48, imu_age_ms, inner.ack)
                if frame_seq is not None:
                    inner.seq = (frame_seq - 1) & 65535
                inner.emit(0x93, struct.pack('<4BI4h4h3h3h3h3h4h4H', *fields))

            def send(inner, data):
                frames = inner.parser.feed(data)
                if len(frames) != 1:
                    raise AssertionError('driver did not emit one valid CRC frame')
                kind, seq, payload = frames[0]
                inner.sent.append((kind, seq, payload))
                if kind == 1:
                    nonce, = struct.unpack('<I', payload)
                    inner.nonce = nonce
                    inner.emit(0x81, struct.pack('<4sBBHHI', b'SLC2', 2, 4, 60000, 500, nonce))
                    inner.status(handshake_flags)
                elif kind == 0x13:
                    nonce, = struct.unpack('<I', payload)
                    inner.emit(0x83, struct.pack('<IBHH', nonce, 1, 750, 0))
                elif kind == 0x11:
                    if fail_pwm:
                        raise TimeoutError('TEST_ONLY serial write failure')
                    _, *pwm = struct.unpack('<I4h', payload)
                    inner.pwm, inner.ack = pwm, seq
                elif kind == 0x14:
                    inner.emit(0x91, struct.pack('<4BIII4h4h4h2B', 1, 1, 1, 1, 0, 0, 0,
                                                *([0]*12), 0, 0))
                elif kind == 0x12:
                    nonce, *signs = struct.unpack('<I8b', payload)
                    if configure_ack:
                        inner.emit(0x82, struct.pack('<IB', nonce if configure_nonce_matches else nonce ^ 1, 1))
                    inner.configured = configured_after_config
                    if fresh_config_status:
                        inner.status(tick=configure_tick_us)
                elif kind == 0x10:
                    if fail_speed and any(struct.unpack('<4h', payload)):
                        raise TimeoutError('TEST_ONLY speed serial write failure')
                    # Targets are not actual speed/PWM: this harness has no PI or motor.
                    inner.target_speed, inner.ack = list(struct.unpack('<4h', payload)), seq
                elif kind == 0x15:
                    if struct.unpack('<I', payload) != (inner.nonce,):
                        raise AssertionError('register request must use the active session nonce')
                    if fail_motor_register_request:
                        raise TimeoutError('TEST_ONLY motor register request write failure')
                    if motor_register_reply:
                        inner.emit(0x92, struct.pack('<15I', *REGISTER_VALUES))
                elif kind == 0x17:
                    if struct.unpack('<I', payload) != (inner.nonce,):
                        raise AssertionError('body query must use the active session nonce')
                    inner.body_started = True
                    if body_capability:
                        inner.emit(0x84, struct.pack('<IBBBB',
                                   inner.nonce if body_nonce_matches else inner.nonce ^ 1,
                                   body_version, body_who, body_startup_imu_flags, 1))
                        if body_startup_feedback:
                            inner.body_status(imu_flags=body_startup_imu_flags,
                                              tick=body_startup_tick_us)
                elif kind == 0x16:
                    nonce, vx, wz = struct.unpack('<Ihh', payload)
                    if nonce != inner.nonce:
                        raise AssertionError('body target must use the active session nonce')
                    if fail_body and (vx or wz):
                        raise TimeoutError('TEST_ONLY body serial write failure')
                    inner.target_body, inner.ack = [vx, wz], seq

            def close(inner):
                inner.closed = True
                os.close(inner.fd)
                os.close(inner.writer)

        class Publisher:
            def __init__(inner, topic, message_type, **kwargs):
                inner.messages = []
                publishers[topic] = inner
                inner.topic = topic

            def publish(inner, message):
                if inner.topic == '~ready':
                    ready_attempts[0] += 1
                    if ready_attempts[0] == ready_fail_at:
                        raise ValueError('TEST_ONLY ready transport failure')
                if inner.topic == '~diagnostics':
                    diagnostic_attempts[0] += 1
                    if diagnostics_fail or diagnostic_attempts[0] == diagnostics_fail_at:
                        raise RuntimeError('TEST_ONLY diagnostic transport failure')
                    if not diagnostics_subscribed[0]:
                        return
                inner.messages.append(message)

        def subscribe(topic, message_type, callback, **kwargs):
            callbacks[topic] = callback
            return NS()

        def shutdown():
            index[0] += 1
            i = index[0]
            if i >= len(commands):
                return True
            diagnostics_subscribed[0] = True
            if i == 0 and config_after_start is not None:
                config_file.write_text(yaml.safe_dump(config_after_start), encoding='utf-8')
            if commands[i] is not None:
                if concurrent_commands:
                    barrier = threading.Barrier(concurrent_commands)

                    def concurrent_receive():
                        barrier.wait(timeout=5)
                        callbacks[cfg['output_topic']](twist(*commands[i]))

                    workers = [threading.Thread(target=concurrent_receive)
                               for _ in range(concurrent_commands)]
                    for worker in workers:
                        worker.start()
                    for worker in workers:
                        worker.join(timeout=5)
                    if any(worker.is_alive() for worker in workers):
                        raise AssertionError('TEST_ONLY concurrent callbacks did not finish')
                callbacks[cfg['output_topic']](twist(*commands[i]))
            if links and (telemetry is None or telemetry[i]):
                for _ in range(1 if burst_telemetry is None else burst_telemetry[i]):
                    links[0].status(None if flags is None else flags[i])
            if links and register_frames_each_loop:
                links[0].emit(0x92, struct.pack('<15I', *REGISTER_VALUES))
            if links and links[0].body_started and (body_telemetry is None or body_telemetry[i]):
                links[0].body_status(imu_flags=7 if body_imu_flags is None else body_imu_flags[i],
                                     imu_age_ms=0 if body_imu_age_ms is None else body_imu_age_ms[i],
                                     tick=None if body_ticks is None else body_ticks[i],
                                     frame_seq=None if body_frame_seq is None else body_frame_seq[i])
            return False

        def sleep(delay):
            # Driver close also sleeps; keep its original delays observable.
            clock[0] += interval if abs(delay-.05) < 1e-12 else delay

        with tempfile.TemporaryDirectory(prefix='ugv-driver-test-') as temp:
            config_file = Path(temp) / 'driver.yaml'
            config_file.write_text(yaml.safe_dump(cfg), encoding='utf-8')

            def get_param(name, *default):
                if name == '~config_file' and not omit_config:
                    return str(config_file)
                if ros_params is not None and name in ros_params:
                    return ros_params[name]
                if default:
                    return default[0]
                raise KeyError(name)

            def set_param(name, item):
                set_param_attempts[0] += 1
                if set_param_attempts[0] == set_param_fail_at:
                    raise RuntimeError('TEST_ONLY parameter transport failure')
                xmlrpc.client.dumps((item,), allow_none=True)
                # A ROS parameter write transports a value, not a shared dict.
                params[name] = copy.deepcopy(item)

            driver.rospy = NS(init_node=lambda *a, **k: None, get_param=get_param,
                              set_param=set_param,
                              Subscriber=subscribe, Publisher=Publisher, is_shutdown=shutdown,
                              Time=NS(now=lambda: clock[0]), get_name=lambda: '/slicer_c30d_driver',
                              ROSException=RuntimeError, logwarn=lambda *a: warn_logs.append(a),
                              loginfo=lambda *args: info_logs.append(args),
                              logwarn_throttle=lambda *a: warn_logs.append(a), logfatal=lambda *a: None)
            driver.tf2_ros = NS(TransformBroadcaster=lambda: NS(sendTransform=lambda msg: None))
            error = None
            select_now = link_module.select.select

            def select_clock(readers, writers, errors, timeout):
                ready = select_now(readers, writers, errors, 0)
                if not ready[0] and timeout:
                    clock[0] += timeout
                return ready

            with patch.object(link_module, 'Link', FakeLink), \
                    patch.object(link_module, 'identity', return_value=('1a86', '55d4', 'TEST_ONLY')), \
                    patch.object(link_module.glob, 'glob', return_value=['/TEST_ONLY_SERIAL']), \
                    patch.object(driver.time, 'monotonic', side_effect=lambda: clock[0]), \
                    patch.object(driver.time, 'sleep', side_effect=sleep), \
                    patch.object(link_module.select, 'select', side_effect=select_clock) if cfg.get('control_mode') == 'body_speed' or not (configure_ack and configure_nonce_matches and configured_after_config and fresh_config_status and motor_register_reply and not register_frames_each_loop) else contextlib.nullcontext(), \
                    contextlib.redirect_stdout(__import__('io').StringIO()):
                try:
                    driver.main()
                except (RuntimeError, KeyError, TimeoutError, ValueError) as exc:
                    error = exc
        records = []
        for array in publishers.get('~diagnostics', NS(messages=[])).messages:
            for status in array.status:
                records.append((status.message, {kv.key: value(kv.value) for kv in status.values}))
        frames = links[0].sent if links else []
        output = [list(struct.unpack('<I4h', payload)[1:]) for kind, seq, payload in frames if kind == 0x11]
        speed_output = [list(struct.unpack('<4h', payload)) for kind, seq, payload in frames if kind == 0x10]
        body_output = [list(struct.unpack('<Ihh', payload)[1:]) for kind, seq, payload in frames if kind == 0x16]
        return NS(driver=driver, error=error, output=output, speed_output=speed_output, body_output=body_output, records=records, frames=frames,
                  links=links, publishers=publishers, telemetry_count=telemetry_count[0],
                  params=params, info_logs=info_logs, warn_logs=warn_logs,
                  diagnostic_attempts=diagnostic_attempts[0],
                  set_param_attempts=set_param_attempts[0])

    def assert_clean(self, run):
        self.assertIsNone(run.error)
        self.assertTrue(run.links[0].closed)
        self.assertIs(run.publishers['~ready'].messages[-1], False)

    def test_actual_driver_import_and_continuous_zero_crossing(self):
        for w in (0., 1e-9, 1.00001e-9, 1e-6, -1e-9, -1.00001e-9, -1e-6):
            with self.subTest(w=w):
                run = self.run_driver([(.125, w)]*12)
                self.assert_clean(run)
                self.assertEqual(run.output[-1], [142, -142, 142, -142])

    def test_mixed_command_preserves_geometric_ratio(self):
        run = self.run_driver([(.125, .25)]*12)
        self.assert_clean(run)
        self.assertEqual(run.output[-1], [213, -157, 213, -157])

    def test_straight_and_pure_turn_endpoints(self):
        for command, expected in [((.22, 0), [250, -250, 250, -250]),
                                  ((0, .25), [71]*4), ((0, -.25), [-71]*4)]:
            with self.subTest(command=command):
                run = self.run_driver([command]*12)
                self.assert_clean(run)
                self.assertEqual(run.output[-1], expected)

    def test_saturation_and_extreme_finite_values(self):
        for command in ((1e308, 1e308), (1e308, 0), (0, 1e308)):
            with self.subTest(command=command):
                run = self.run_driver([command]*20)
                self.assert_clean(run)
                self.assertEqual(max(map(abs, run.output[-1])), 600)
                self.assertTrue(all(abs(p) <= 600 for row in run.output for p in row))

    def test_zero_is_immediate_and_reversal_has_zero_cycle(self):
        run = self.run_driver([(.22, 0)]*6 + [(0, 0), (.22, 0), (-.22, 0), (-.22, 0)])
        self.assert_clean(run)
        self.assertEqual(run.output[5], [250, -250, 250, -250])
        self.assertEqual(run.output[6], [0]*4)
        self.assertEqual(run.output[7], [50, -50, 50, -50])
        self.assertEqual(run.output[8], [0]*4)
        self.assertEqual(run.output[9], [-50, 50, -50, 50])

    def test_nonfinite_input_invalidates_command_immediately(self):
        for invalid in ((math.nan, 0), (0, math.inf), (-math.inf, 0)):
            with self.subTest(invalid=invalid):
                run = self.run_driver([(.22, 0)]*6 + [invalid, None])
                self.assert_clean(run)
                self.assertEqual(run.output[-2:], [[0]*4, [0]*4])

    def test_command_watchdog_at_exact_boundary(self):
        run = self.run_driver([(.22, 0), None, None], interval=.25)
        self.assert_clean(run)
        self.assertEqual(run.output, [[0]*4]*3)
        reasons = [data['stop_reason'] for name, data in run.records if name == 'command_tx']
        self.assertEqual(reasons[1:], ['command_timeout', 'command_timeout'])

    def test_slew_clamps_long_dt_to_point_one_seconds(self):
        run = self.run_driver([(.22, 0)]*3, interval=.2)
        self.assert_clean(run)
        self.assertEqual(run.output, [[0]*4, [100, -100, 100, -100], [200, -200, 200, -200]])

    def test_telemetry_delay_zeros_then_fresh_data_can_resume(self):
        run = self.run_driver([(.22, 0)]*4, telemetry=[True, False, False, True], interval=.2)
        self.assert_clean(run)
        self.assertEqual(run.output[2], [0]*4)
        self.assertEqual(run.output[3], [100, -100, 100, -100])
        reasons = [data['stop_reason'] for name, data in run.records if name == 'command_tx']
        self.assertEqual(reasons[2], 'telemetry_delayed')

    def test_telemetry_hard_timeout_stops_and_closes(self):
        run = self.run_driver([(.22, 0)]*5, telemetry=[True, False, False, False, False], interval=.2)
        self.assertIsInstance(run.error, RuntimeError)
        self.assertIn('telemetry timeout', str(run.error))
        self.assertTrue(run.links[0].closed)
        self.assertEqual([kind for kind, seq, data in run.frames[-10:]], [0x10]*10)
        self.assertTrue(all(data == b'\0'*8 for kind, seq, data in run.frames[-10:]))

    def test_fault_and_serial_write_error_keep_finally_stop(self):
        for options in (dict(flags=[52]), dict(fail_pwm=True)):
            with self.subTest(options=options):
                run = self.run_driver([(.22, 0)], **options)
                self.assertIsNotNone(run.error)
                self.assertTrue(run.links[0].closed)
                self.assertEqual([kind for kind, seq, data in run.frames[-10:]], [0x10]*10)
                self.assertTrue(any(name == 'fault' for name, data in run.records))

    def test_diagnostic_failure_does_not_prevent_zero_or_close(self):
        run = self.run_driver([(.22, 0)]*6 + [(0, 0)], diagnostics_fail=True)
        self.assert_clean(run)
        self.assertEqual(run.output[-1], [0]*4)

    def test_explicit_config_required_before_opening_serial(self):
        run = self.run_driver([], omit_config=True)
        self.assertIsNotNone(run.error)
        self.assertEqual(run.links, [])

    def test_telemetry_every_frame_and_units_are_recorded(self):
        run = self.run_driver([(.22, 0)]*8, burst_telemetry=[3]+[1]*7)
        self.assert_clean(run)
        rx = [data for name, data in run.records if name == 'telemetry']
        self.assertEqual(len(rx), run.telemetry_count)
        self.assertEqual(len(rx), 11, 'multiple frames in one read must not overwrite each other')
        self.assertEqual(rx[-1]['pwm_timer_ticks_abcd'], [400, -400, 400, -400])
        self.assertEqual(rx[-1]['voltage_v'], 12.5)
        self.assertEqual(rx[-1]['raw_encoder_abcd'], [0]*4)
        self.assertIn('mcu_tick_us', rx[-1])
        self.assertIn('mcu_frame_seq', rx[-1])
        self.assertIn('host_monotonic_s', rx[-1])
        transmitted = {seq for kind, seq, data in run.frames if kind == 0x11}
        self.assertIn(rx[-1]['accepted_command_seq'], transmitted)

    def test_telemetry_exact_hard_timeout_boundary_still_sends_zero(self):
        run = self.run_driver([(.22, 0)]*4, telemetry=[True, False, False, False], interval=.25)
        self.assertIn('telemetry timeout', str(run.error))
        # At .25s the original reread threshold is not exceeded; at .50s the
        # hard timeout is not exceeded yet, and the delayed-telemetry zero wins.
        self.assertEqual(run.output, [[0]*4, [100, -100, 100, -100], [0]*4])

    def test_pwm_frame_golden_bytes_sequence_wrap_and_nonce(self):
        driver = self.load_driver()
        car = driver.Chassis.__new__(driver.Chassis)
        car.cfg, car.seq, car.nonce = dict(CONFIG), 65534, 0x12345678
        sent = []
        car.link, car.observe = NS(send=sent.append), None
        self.assertEqual(car.pwm([142, -142, 142, -142]), 65535)
        # CRC-CCITT independently checked with binascii.crc_hqx(body, 0xffff).
        self.assertEqual(sent[0].hex(), 'a55a01110cffff785634128e0072ff8e0072ffc45b')
        self.assertEqual(car.pwm([0]*4), 0)
        self.assertEqual(sent[1][5:7], b'\0\0')
        with self.assertRaises(ValueError):
            car.pwm([751, 0, 0, 0])

    def test_parser_fragment_crc_and_recovery_do_not_create_telemetry(self):
        driver = self.load_driver()
        parser = driver.chassis_core.Parser()
        good = bytes.fromhex('a55a01110cffff785634128e0072ff8e0072ffc45b')
        bad = good[:-1] + bytes([good[-1] ^ 1])
        self.assertEqual(parser.feed(b'noise'+bad+good[:4]), [])
        self.assertEqual(parser.bad_crc, 1)
        self.assertEqual(parser.feed(good[4:]), [(0x11, 65535, bytes.fromhex('785634128e0072ff8e0072ff'))])

    def test_tick_wrap_and_repeated_tick_remain_raw_observations(self):
        driver = self.load_driver()
        car = driver.Chassis.__new__(driver.Chassis)
        reader, writer = os.pipe()
        self.addCleanup(os.close, reader)
        self.addCleanup(os.close, writer)
        records = []
        car.link, car.parser = NS(fd=reader), driver.chassis_core.Parser()
        car.calibration_frames, car.last_rx = 0, 0.
        car.observe = lambda event, **data: records.append((event, data))
        for seq, tick, ack in ((65535, 0xffffff00, 65535), (0, 100, 0), (1, 100, 0)):
            fields = (tick, 50000, 0, 0, 0, 0, 48, ack, *([0]*12), 12500)
            os.write(writer, driver.chassis_core.packet(0x90, seq, struct.pack('<II4iBH4i4h4hH', *fields)))
        car.poll(0)
        self.assertEqual([data['mcu_frame_seq'] for name, data in records], [65535, 0, 1])
        self.assertEqual([data['mcu_tick_us'] for name, data in records], [0xffffff00, 100, 100])
        self.assertEqual([data['accepted_command_seq'] for name, data in records], [65535, 0, 0])

    def test_tx_diagnostics_report_actual_frame_sequence_and_values(self):
        run = self.run_driver([(.125, .25)]*8)
        self.assert_clean(run)
        tx = [data for name, data in run.records if name == 'command_tx']
        frames = [(seq, list(struct.unpack('<I4h', data)[1:]))
                  for kind, seq, data in run.frames if kind == 0x11]
        self.assertEqual([(data['command_seq'], data['output_pwm_permille_abcd']) for data in tx], frames)
        self.assertEqual(tx[-1]['desired_pwm_permille_abcd'], [213, -157, 213, -157])
        self.assertTrue(all(data['input_age_s'] == 0 for data in tx))
        startup = [data for name, data in run.records if name == 'startup'][0]
        self.assertEqual(startup['pwm_mapping'], 'continuous_open_loop_reference')
        self.assertIn('control_core.py', startup['module_paths_sha256'])
        self.assertIn('slicer_manual.py', startup['module_paths_sha256'])
        self.assertEqual(len(startup['config_sha256']), 64)

    def test_actual_import_identity_remains_readable_after_startup(self):
        run = self.run_driver([(0, 0)])
        self.assert_clean(run)
        self.assertIn('~provenance', run.params, 'late recorders must be able to query startup identity')
        provenance = run.params['~provenance']
        self.assertEqual(provenance['pwm_mapping'], 'continuous_open_loop_reference')
        self.assertNotIn('mcu_configuration', provenance)
        for name in ('control_core.py', 'slicer_manual.py', 'c30d_driver_node.py', 'c30d_link.py'):
            entry = provenance['module_paths_sha256'][name]
            self.assertEqual(Path(entry['path']).resolve(), SCRIPTS / name)
            self.assertEqual(entry['sha256'], __import__('hashlib').sha256((SCRIPTS/name).read_bytes()).hexdigest())
        self.assertTrue(any('provenance' in str(args[0]).lower() for args in run.info_logs))

    def test_effective_config_retains_loaded_values_and_final_tf_override(self):
        run = self.run_driver([(.125, 0)]*8,
                              config=dict(linear_speed=.25, publish_tf=True),
                              ros_params={'~publish_tf': False},
                              config_after_start=dict(CONFIG, linear_speed=.88, publish_tf=True))
        self.assert_clean(run)
        provenance = run.params['~provenance']
        self.assertIn('effective_config', provenance,
                      'late recorders need the configuration actually used, not only its hash')
        self.assertEqual(provenance['effective_config'], dict(CONFIG, linear_speed=.25, publish_tf=False))
        self.assertEqual(run.output[-1], [125, -125, 125, -125])
        startup = next(data for name, data in run.records if name == 'startup')
        self.assertEqual(startup['effective_config'], provenance['effective_config'])

    def test_driver_instances_are_consistent_and_distinct_across_restarts(self):
        runs = [self.run_driver([(0, 0)]) for _ in range(2)]
        identifiers = []
        for run in runs:
            self.assert_clean(run)
            identifier = run.params['~provenance'].get('driver_instance_id')
            self.assertIsInstance(identifier, str, 'each driver run must expose its instance identity')
            self.assertTrue(identifier)
            self.assertEqual({data.get('driver_instance_id') for name, data in run.records}, {identifier})
            identifiers.append(identifier)
        self.assertNotEqual(*identifiers, 'command sequence reuse after restart must be distinguishable')

    def test_event_sequences_are_unique_for_concurrent_command_callbacks(self):
        run = self.run_driver([(.125, 0)]*4, concurrent_commands=16)
        self.assert_clean(run)
        sequences = [data.get('event_seq') for name, data in run.records]
        self.assertTrue(all(type(seq) is int for seq in sequences),
                        'every diagnostic must expose an integer event sequence')
        self.assertEqual(sorted(sequences), list(range(1, len(sequences)+1)))
        self.assertEqual(sum(name == 'command_rx' for name, data in run.records), 68)

    def test_transient_diagnostic_failure_leaves_visible_sequence_gap_and_zero(self):
        run = self.run_driver([(.22, 0)]*6 + [(0, 0)], diagnostics_fail_at=7)
        self.assert_clean(run)
        self.assertEqual(run.output[-1], [0]*4)
        sequences = [data.get('event_seq') for name, data in run.records]
        self.assertTrue(all(type(seq) is int for seq in sequences),
                        'failure must consume an event number before publication')
        self.assertEqual(sequences, [seq for seq in range(1, run.diagnostic_attempts+1) if seq != 7])
        self.assertTrue(any(name == 'shutdown' for name, data in run.records))

    @staticmethod
    def speed_config(**updates):
        return dict(control_mode='wheel_speed', pwm_signs=[-1, 1, 1, 1], **updates)

    @staticmethod
    def body_config(**updates):
        return dict(control_mode='body_speed', pwm_signs=[-1, 1, 1, 1], **updates)

    def assert_body_closed_without_fallback(self, run):
        self.assertTrue(run.links, 'body mode must reach its own serial path: %s' % run.error)
        self.assertTrue(run.links[0].closed)
        self.assertEqual(run.output, [])
        self.assertEqual(run.speed_output, [])
        self.assertEqual(run.body_output[-10:], [[0, 0]]*10)

    def test_body_mode_sends_si_targets_after_configuration_and_capability_without_fallback(self):
        run = self.run_driver([(.05, .1)]*2, config=self.body_config())
        self.assert_clean(run)
        self.assertEqual(run.body_output[:2], [[50, 100]]*2)
        self.assertEqual(run.body_output[-10:], [[0, 0]]*10)
        kinds = [kind for kind, seq, payload in run.frames]
        self.assertNotIn(0x10, kinds)
        self.assertNotIn(0x11, kinds)
        self.assertNotIn(0x13, kinds)
        self.assertLess(kinds.index(0x12), kinds.index(0x17))
        self.assertLess(kinds.index(0x17), kinds.index(0x16))
        self.assertTrue(any(run.publishers['~ready'].messages))

    def test_body_startup_requires_extension_nonce_version_who_and_imu_feedback(self):
        for options in (dict(body_capability=False), dict(body_nonce_matches=False),
                        dict(body_version=2), dict(body_who=0),
                        dict(body_startup_feedback=False), dict(body_startup_imu_flags=3)):
            with self.subTest(options=options):
                run = self.run_driver([(.05, .1)], config=self.body_config(), **options)
                self.assertIsNotNone(run.error)
                self.assert_body_closed_without_fallback(run)
                self.assertTrue(all(row == [0, 0] for row in run.body_output))
                self.assertFalse(any(run.publishers['~ready'].messages))

    def test_body_startup_still_requires_configuration_ack_and_fresh_configured_status(self):
        for options in (dict(configure_ack=False), dict(configure_nonce_matches=False),
                        dict(configured_after_config=False), dict(fresh_config_status=False)):
            with self.subTest(options=options):
                run = self.run_driver([(.05, .1)], config=self.body_config(), **options)
                self.assertIsNotNone(run.error)
                self.assert_body_closed_without_fallback(run)
                self.assertTrue(all(row == [0, 0] for row in run.body_output))
                self.assertFalse(any(run.publishers['~ready'].messages))

    def test_body_targets_bypass_host_wheel_mapping_slew_and_reversal_zero(self):
        run = self.run_driver([(.05, .1), (-.05, -.1), (.05, -.1), (0, 0)],
                              config=self.body_config(track_width=.9,
                                  wheel_ports=dict(left_front='A', right_front='B',
                                                   left_rear='C', right_rear='D')))
        self.assert_clean(run)
        self.assertEqual(run.body_output[:-10], [[50, 100], [-50, -100], [50, -100], [0, 0]])
        self.assert_body_closed_without_fallback(run)

    def test_body_zero_nonfinite_and_command_timeout_send_immediate_body_zero(self):
        run = self.run_driver([(.05, .1), (0, 0), (.05, .1), (math.nan, 0),
                               (.05, .1), (0, math.inf), (.05, .1), None, None],
                              config=self.body_config(), interval=.25)
        self.assert_clean(run)
        self.assertEqual(run.body_output[:-10], [[50, 100], [0, 0], [50, 100], [0, 0],
                                                 [50, 100], [0, 0], [50, 100], [0, 0], [0, 0]])
        self.assert_body_closed_without_fallback(run)
        reasons = [data['stop_reason'] for name, data in run.records if name == 'command_tx']
        self.assertEqual(reasons[-2:], ['command_timeout']*2)

    def test_body_imu_flags_or_age_failure_zeros_and_fresh_feedback_resumes(self):
        for options in (dict(body_imu_flags=[7, 6, 7]), dict(body_imu_flags=[7, 5, 7]),
                        dict(body_imu_flags=[7, 3, 7]), dict(body_imu_age_ms=[100, 101, 100])):
            with self.subTest(options=options):
                run = self.run_driver([(.05, .1)]*3, config=self.body_config(), **options)
                self.assert_clean(run)
                self.assertEqual(run.body_output[:-10], [[50, 100], [0, 0], [50, 100]])
                self.assertEqual(run.publishers['~ready'].messages, [False, True, False, True, False])
                tx = [data for name, data in run.records if name == 'command_tx']
                self.assertEqual(tx[1]['stop_reason'], 'body_feedback_unavailable')
                self.assert_body_closed_without_fallback(run)

    def test_body_feedback_timeout_is_independent_of_continuing_wheel_telemetry(self):
        run = self.run_driver([(.05, .1)]*5, config=self.body_config(), interval=.2,
                              body_telemetry=[True, False, False, False, True])
        self.assert_clean(run)
        self.assertEqual(run.body_output[:-10], [[50, 100], [50, 100], [0, 0], [0, 0], [50, 100]])
        self.assertEqual(sum(name == 'telemetry' for name, data in run.records), run.telemetry_count)
        tx = [data for name, data in run.records if name == 'command_tx']
        self.assertEqual([data['stop_reason'] for data in tx[2:4]], ['body_feedback_unavailable']*2)
        self.assert_body_closed_without_fallback(run)

    def test_body_repeated_mcu_time_cannot_be_kept_alive_by_fresh_headers(self):
        run = self.run_driver([(.05, .1)]*5, config=self.body_config(), interval=.2,
                              body_startup_tick_us=10_000_000,
                              body_ticks=[10_000_000]*4 + [10_800_000])
        self.assert_clean(run)
        self.assertEqual(run.body_output[:-10], [[50, 100], [50, 100], [0, 0], [0, 0], [50, 100]])
        body = [data for name, data in run.records if name == 'body_telemetry']
        self.assertEqual(len({data['mcu_frame_seq'] for data in body}), len(body))
        self.assert_body_closed_without_fallback(run)

    def test_body_header_and_mcu_time_wrap_continue_accepting_fresh_targets(self):
        run = self.run_driver([(.05, .1)]*3, config=self.body_config(),
                              body_startup_tick_us=0xfffffe00,
                              body_ticks=[0xffffff00, 100, 50_100],
                              body_frame_seq=[65535, 0, 1])
        self.assert_clean(run)
        self.assertEqual(run.body_output[:-10], [[50, 100]]*3)
        body = [data for name, data in run.records if name == 'body_telemetry']
        self.assertEqual([data['mcu_frame_seq'] for data in body[-3:]], [65535, 0, 1])
        self.assert_body_closed_without_fallback(run)

    def test_body_feedback_limits_use_explicit_config_values(self):
        run = self.run_driver([(.05, .1)]*3,
                              config=self.body_config(body_feedback_timeout_s=.15,
                                                      body_imu_timeout_ms=10),
                              body_imu_age_ms=[10, 11, 10])
        self.assert_clean(run)
        self.assertEqual(run.body_output[:-10], [[50, 100], [0, 0], [50, 100]])
        run = self.run_driver([(.05, .1)]*2, interval=.2,
                              config=self.body_config(body_feedback_timeout_s=.15),
                              body_telemetry=[True, False])
        self.assert_clean(run)
        self.assertEqual(run.body_output[:-10], [[50, 100], [0, 0]])

    def test_body_fault_or_configuration_loss_stops_in_same_mode(self):
        for flag in (48, 56 | 64, 56 | 128, 56 | 4):
            with self.subTest(flag=flag):
                run = self.run_driver([(.05, .1)]*2, config=self.body_config(), flags=[56, flag])
                self.assertIsInstance(run.error, RuntimeError)
                self.assertEqual(run.body_output[0], [50, 100])
                self.assertTrue(all(row == [0, 0] for row in run.body_output[1:]))
                self.assertIs(run.publishers['~ready'].messages[-1], False)
                self.assert_body_closed_without_fallback(run)

    def test_body_motor_off_or_power_missing_is_normal_all_zero_and_not_ready(self):
        for flag in (8, 8 | 16, 8 | 32):
            with self.subTest(flag=flag):
                run = self.run_driver([(.05, .1)]*2, config=self.body_config(), flags=[flag]*2)
                self.assert_clean(run)
                self.assertTrue(all(row == [0, 0] for row in run.body_output))
                self.assertFalse(any(run.publishers['~ready'].messages))
                self.assert_body_closed_without_fallback(run)

    def test_body_serial_and_observation_failures_keep_explicit_zero_and_close(self):
        for options in (dict(fail_body=True), dict(diagnostics_fail=True), dict(ready_fail_at=4)):
            with self.subTest(options=options):
                run = self.run_driver([(.05, .1)]*2, config=self.body_config(), **options)
                if options.get('fail_body'):
                    self.assertIsInstance(run.error, TimeoutError)
                else:
                    self.assertIsNone(run.error)
                self.assert_body_closed_without_fallback(run)

    def test_body_fault_sends_zero_before_diagnostic_when_ready_publish_fails(self):
        run = self.run_driver([(.05, .1)]*2, config=self.body_config(), flags=[56, 60], ready_fail_at=3)
        self.assertIsInstance(run.error, RuntimeError)
        fault_request = next(i for i, frame in enumerate(run.frames) if frame[0] == 0x14)
        kind, seq, payload = run.frames[fault_request-1]
        self.assertEqual(kind, 0x16)
        self.assertEqual(struct.unpack('<Ihh', payload)[1:], (0, 0))
        self.assert_body_closed_without_fallback(run)

    def test_body_diagnostics_record_actual_targets_ack_and_original_wheel_odometry(self):
        run = self.run_driver([(.05, .1)]*3, config=self.body_config())
        self.assert_clean(run)
        self.assertEqual(run.params['~provenance']['control_mode'], 'body_speed')
        tx = [data for name, data in run.records if name == 'command_tx']
        self.assertEqual([(data['target_body_vx_mm_s'], data['target_body_wz_mrad_s']) for data in tx],
                         [(50, 100)]*3)
        serial = [data for name, data in run.records if name == 'serial_tx' and data['kind'] == 0x16]
        self.assertEqual((serial[0]['body_vx_mm_s'], serial[0]['body_wz_mrad_s']), (50, 100))
        body = [data for name, data in run.records if name == 'body_telemetry']
        transmitted = {seq for kind, seq, payload in run.frames if kind == 0x16}
        self.assertIn(body[-1]['ack_seq'], transmitted)
        self.assertEqual((body[-1]['vx_m_s'], body[-1]['wz_rad_s']), (.011, .022))
        wheel = [data for name, data in run.records if name == 'telemetry']
        self.assertTrue(all(data['measured_mm_s_abcd'] == [0]*4 for data in wheel))
        odom = run.publishers[CONFIG['odom_topic']].messages
        self.assertTrue(odom)
        self.assertTrue(all(msg.twist.twist.linear.x == 0 and msg.twist.twist.angular.z == 0 for msg in odom))

    def test_body_requires_independent_signs_before_serial_and_mock_opens_no_serial(self):
        run = self.run_driver([], config=dict(control_mode='body_speed'))
        self.assertIsInstance(run.error, ValueError)
        self.assertEqual(run.links, [])
        run = self.run_driver([(.05, .1)]*2, config=self.body_config(), ros_params={'~mock': True})
        self.assertIsNone(run.error)
        self.assertEqual(run.links, [])
        self.assertEqual(run.frames, [])

    def test_speed_mode_configures_independent_signs_before_speed_targets(self):
        run = self.run_driver([(.125, 0)]*3, config=self.speed_config())
        self.assert_clean(run)
        self.assertEqual(run.output, [], 'wheel-speed branch must never send PWM frames')
        self.assertNotIn(0x13, [kind for kind, seq, data in run.frames])
        configure = [(seq, struct.unpack('<I8b', data)) for kind, seq, data in run.frames if kind == 0x12]
        self.assertEqual(len(configure), 1)
        self.assertEqual(list(configure[0][1][1:]), [-1, 1, 1, 1, -1, 1, -1, 1])
        self.assertEqual(run.speed_output[:3], [[125]*4]*3)
        self.assertEqual(run.speed_output[-10:], [[0]*4]*10)
        first_speed = next(i for i, frame in enumerate(run.frames) if frame[0] == 0x10)
        self.assertLess(next(i for i, frame in enumerate(run.frames) if frame[0] == 0x12), first_speed)
        self.assertTrue(any(run.publishers['~ready'].messages))

    def test_speed_configuration_is_readable_after_startup_diagnostics_are_missed(self):
        with patch('secrets.randbits', return_value=0xfedcba98):
            run = self.run_driver([(0, 0)], config=self.speed_config(), flags=[8],
                                  diagnostics_subscribe_after_start=True, configure_tick_us=0xf1234567)
        self.assert_clean(run)
        self.assertFalse(any(name in ('startup', 'configuration_ack', 'mcu_configuration')
                             or (name == 'serial_tx' and data['kind'] == 0x12)
                             for name, data in run.records))
        provenance = run.params['~provenance']
        self.assertIn('mcu_configuration', provenance,
                      'a late recorder must recover the completed configuration from ROS parameters')
        request, = [struct.unpack('<I8b', data) for kind, seq, data in run.frames if kind == 0x12]
        snapshot = provenance['mcu_configuration']
        self.assertEqual(snapshot['request_nonce'], str(request[0]))
        self.assertEqual(snapshot['ack_nonce'], str(request[0]))
        self.assertEqual(int(snapshot['request_nonce']), 0xfedcba98)
        self.assertEqual(snapshot['accepted'], 1)
        self.assertEqual(snapshot['pwm_signs_abcd'], list(request[1:5]))
        self.assertEqual(snapshot['encoder_signs_abcd'], list(request[5:]))
        self.assertEqual(snapshot['pwm_signs_abcd'], [-1, 1, 1, 1])
        self.assertEqual(snapshot['encoder_signs_abcd'], [-1, 1, -1, 1])
        self.assertIs(snapshot['configured'], True)
        self.assertEqual(snapshot['mcu_tick_us'], str(0xf1234567))
        xmlrpc.client.dumps((snapshot,))
        self.assertEqual({data['driver_instance_id'] for name, data in run.records},
                         {provenance['driver_instance_id']})
        self.assertTrue(all(row == [0]*4 for row in run.speed_output))
        self.assertFalse(any(run.publishers['~ready'].messages))

    def test_speed_configuration_success_diagnostic_matches_persisted_snapshot(self):
        run = self.run_driver([(0, 0)], config=self.speed_config())
        self.assert_clean(run)
        events = [data for name, data in run.records if name == 'mcu_configuration']
        self.assertEqual(len(events), 1, 'online observers need a completed configuration event')
        provenance = run.params['~provenance']
        expected = dict(provenance['mcu_configuration'])
        # Diagnostic KeyValue strings are decoded as JSON scalars by this helper.
        for key in ('request_nonce', 'ack_nonce', 'mcu_tick_us'):
            expected[key] = int(expected[key])
        self.assertEqual({key: events[0][key] for key in expected}, expected)
        self.assertEqual(events[0]['driver_instance_id'], provenance['driver_instance_id'])

    def test_speed_configuration_snapshot_write_failure_cannot_prevent_zero_or_close(self):
        run = self.run_driver([(0, 0)], config=self.speed_config(), set_param_fail_at=2)
        self.assert_clean(run)
        self.assertEqual(run.set_param_attempts, 2, 'the completed snapshot must be written to ROS')
        self.assertNotIn('mcu_configuration', run.params['~provenance'])
        self.assertTrue(any(name == 'mcu_configuration' for name, data in run.records))
        self.assertEqual(run.speed_output, [[0]*4]*11)
        self.assertEqual(run.output, [])

    def test_speed_mapping_uses_si_units_ports_and_preserves_saturation_ratio(self):
        for command, expected in [((0, .25), [19, -19, 19, -19]),
                                  ((.125, .25), [144, 106, 144, 106]),
                                  ((1, 8), [600, 150, 600, 150]),
                                  ((-1, -8), [-600, -150, -600, -150]),
                                  ((.0015, 0), [2]*4),
                                  ((1e308, 1e308), [600, 516, 600, 516]),
                                  ((0, -1e308), [-600, 600, -600, 600])]:
            with self.subTest(command=command):
                run = self.run_driver([command], config=self.speed_config())
                self.assert_clean(run)
                self.assertEqual(run.speed_output[0], expected)
                self.assertEqual(run.output, [])

    def test_speed_port_order_changes_wire_targets_not_polarity(self):
        config = self.speed_config(wheel_ports=dict(left_front='A', right_front='B',
                                                    left_rear='C', right_rear='D'))
        run = self.run_driver([(0, .25)], config=config)
        self.assert_clean(run)
        self.assertEqual(run.speed_output[0], [-19, 19, -19, 19])

    def test_speed_zero_watchdog_invalid_and_reversal_stop_without_pwm_slew(self):
        run = self.run_driver([(.125, 0), (0, 0), (.125, 0), (-.125, 0),
                               (-.125, 0), (math.nan, 0), (.125, 0), None, None],
                              config=self.speed_config(), interval=.25)
        self.assert_clean(run)
        self.assertEqual(run.speed_output[:-10], [[125]*4, [0]*4, [125]*4, [0]*4,
                                                   [-125]*4, [0]*4, [125]*4, [0]*4, [0]*4])

    def test_speed_startup_requires_ack_matching_nonce_and_fresh_configured_status(self):
        for options in (dict(configure_ack=False), dict(configure_nonce_matches=False),
                        dict(configured_after_config=False), dict(fresh_config_status=False),
                        dict(fresh_config_status=False, handshake_flags=56)):
            with self.subTest(options=options):
                run = self.run_driver([(.125, 0)], config=self.speed_config(), **options)
                self.assertIsNotNone(run.error)
                self.assertEqual(run.output, [])
                self.assertTrue(all(not any(row) for row in run.speed_output))
                self.assertTrue(run.links[0].closed)
                self.assertFalse(any(run.publishers['~ready'].messages))
                self.assertNotIn('mcu_configuration', run.params['~provenance'])
                self.assertFalse(any(name == 'mcu_configuration' for name, data in run.records))

    def test_speed_config_loss_direct_or_calibration_fails_stopped_no_fallback(self):
        for flag in (48, 56 | 128, 56 | 64, 56 | 4):
            with self.subTest(flags=flag):
                run = self.run_driver([(.125, 0)]*2, config=self.speed_config(), flags=[56, flag])
                self.assertIsInstance(run.error, RuntimeError)
                self.assertEqual(run.speed_output[0], [125]*4)
                self.assertGreaterEqual(len(run.speed_output[1:]), 10)
                self.assertTrue(all(row == [0]*4 for row in run.speed_output[1:]))
                self.assertEqual(run.output, [])
                self.assertIs(run.publishers['~ready'].messages[-1], False)

    def test_speed_not_ready_without_enable_or_power_and_no_nonzero_commands(self):
        for flag in (8, 8 | 16, 8 | 32):
            with self.subTest(flags=flag):
                run = self.run_driver([(.125, 0)], config=self.speed_config(), flags=[flag])
                self.assert_clean(run)
                self.assertTrue(all(not any(row) for row in run.speed_output))
                self.assertFalse(any(run.publishers['~ready'].messages))

    def test_speed_diagnostics_separate_requested_targets_from_observed_pwm_and_speed(self):
        run = self.run_driver([(0, .25)]*2, config=self.speed_config())
        self.assert_clean(run)
        provenance = run.params['~provenance']
        self.assertEqual(provenance['control_mode'], 'wheel_speed')
        self.assertIs(provenance['host_pwm_limit_enforced'], False)
        tx = [item for name, item in run.records if name == 'command_tx']
        self.assertEqual(tx[-1]['target_speed_mm_s_abcd'], [19, -19, 19, -19])
        self.assertNotIn('output_pwm_permille_abcd', tx[-1])
        self.assertNotIn('desired_pwm_permille_abcd', tx[-1])
        serial = [item for name, item in run.records if name == 'serial_tx' and item['kind'] == 0x10]
        self.assertEqual(serial[0]['speed_mm_s_abcd'], tx[0]['target_speed_mm_s_abcd'])
        telemetry = [item for name, item in run.records if name == 'telemetry']
        self.assertTrue(all(item['measured_mm_s_abcd'] == [0]*4 for item in telemetry))
        self.assertTrue(all(item['pwm_timer_ticks_abcd'] == [0]*4 for item in telemetry))

    def test_speed_error_and_diagnostic_failure_still_send_explicit_zero(self):
        for options in (dict(fail_speed=True), dict(diagnostics_fail=True)):
            with self.subTest(options=options):
                run = self.run_driver([(.125, 0), (0, 0)], config=self.speed_config(), **options)
                if options.get('fail_speed'):
                    self.assertIsInstance(run.error, TimeoutError)
                else:
                    self.assert_clean(run)
                    self.assertIn('mcu_configuration', run.params['~provenance'])
                self.assertEqual(run.speed_output[-10:], [[0]*4]*10)
                self.assertEqual(run.output, [])

    def test_invalid_mode_missing_independent_signs_and_unenforceable_low_pwm_cap_fail_before_open(self):
        for config in (dict(control_mode='unknown'), dict(control_mode='wheel_speed'),
                       self.speed_config(ros_driver_pwm_limit_permille=500),
                       self.speed_config(max_wheel_mm_s=601),
                       dict(control_mode='wheel_speed', pwm_signs=[1, 1, 1, True])):
            with self.subTest(config=config):
                run = self.run_driver([], config=config)
                self.assertIsInstance(run.error, ValueError)
                self.assertEqual(run.links, [], 'invalid speed configuration must not open the serial port')

    def test_speed_ready_publish_failure_cannot_skip_final_zero_and_serial_close(self):
        for config in ({}, self.speed_config()):
            with self.subTest(mode=config.get('control_mode', 'pwm_reference')):
                run = self.run_driver([(.125, 0)]*2, config=config, ready_fail_at=4)
                self.assertIsNone(run.error, 'failure of an observational ready topic must not prevent stop')
                self.assertTrue(run.links[0].closed)
                self.assertEqual(run.speed_output[-10:], [[0]*4]*10)

    def test_speed_fault_stops_before_diagnostic_even_if_ready_transport_fails(self):
        run = self.run_driver([(.125, 0)]*2, config=self.speed_config(),
                              flags=[56, 60], ready_fail_at=3)
        self.assertIsInstance(run.error, RuntimeError)
        self.assertIn('MCU fault', str(run.error))
        self.assertTrue(run.links[0].closed)
        fault_request = next(i for i, frame in enumerate(run.frames) if frame[0] == 0x14)
        stop_before_fault_request = [data for kind, seq, data in run.frames[:fault_request] if kind == 0x10]
        self.assertEqual(stop_before_fault_request[-1], b'\0'*8)
        self.assertEqual(run.speed_output[-10:], [[0]*4]*10)

    def test_speed_delayed_telemetry_and_hard_timeout_send_only_zero_after_delay(self):
        run = self.run_driver([(.125, 0)]*5, config=self.speed_config(),
                              telemetry=[True, False, False, False, False], interval=.2)
        self.assertIsInstance(run.error, RuntimeError)
        self.assertIn('telemetry timeout', str(run.error))
        self.assertEqual(run.speed_output[:3], [[125]*4, [125]*4, [0]*4])
        self.assertTrue(all(not any(row) for row in run.speed_output[2:]))
        self.assertEqual(run.output, [])
        tx = [item for name, item in run.records if name == 'command_tx']
        self.assertEqual(tx[2]['stop_reason'], 'telemetry_delayed')

    def test_speed_frame_golden_bytes_rejects_non_integer_or_out_of_range(self):
        driver = self.load_driver()
        car = driver.Chassis.__new__(driver.Chassis)
        car.seq, car.max_wheel_mm_s, car.observe = 65534, 600, None
        sent = []
        car.link = NS(send=sent.append)
        self.assertEqual(car.speed([125, -125, 19, -19]), 65535)
        body = bytes.fromhex('011008ffff7d0083ff1300edff')
        import binascii
        expected = b'\xa5\x5a' + body + struct.pack('<H', binascii.crc_hqx(body, 0xffff))
        self.assertEqual(sent, [expected])
        for invalid in ([601, 0, 0, 0], [1.5, 0, 0, 0], [math.nan, 0, 0, 0],
                        [True, 0, 0, 0], [0]*3):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    car.speed(invalid)
        self.assertEqual(len(sent), 1)
        self.assertEqual(car.speed([0]*4), 0)

    def test_launch_override_requires_valid_independent_configuration(self):
        run = self.run_driver([(.125, 0)], config=dict(pwm_signs=[-1, 1, 1, 1]),
                              ros_params={'~control_mode': 'wheel_speed'})
        self.assert_clean(run)
        self.assertEqual(run.speed_output[0], [125]*4)
        self.assertEqual(run.output, [])

    def test_motor_register_request_uses_nonce_without_waiting_for_reply(self):
        driver = self.load_driver()
        car = driver.Chassis.__new__(driver.Chassis)
        car.seq, car.nonce, car.observe = 65534, 0x12345678, None
        sent = []
        car.link = NS(send=sent.append)
        car.wait = lambda *args: self.fail('a register request must not wait for a reply')
        self.assertTrue(callable(getattr(car, 'request_motor_registers', None)),
                        'the serial owner needs a send-only register request')
        self.assertEqual(car.request_motor_registers(), 65535)
        import binascii
        body = bytes.fromhex('011504ffff78563412')
        expected = b'\xa5\x5a' + body + struct.pack('<H', binascii.crc_hqx(body, 0xffff))
        self.assertEqual(sent, [expected])
        self.assertEqual(car.request_motor_registers(), 0)

    def test_motor_register_replies_record_each_actual_frame_without_telemetry_freshness(self):
        driver = self.load_driver()
        car = driver.Chassis.__new__(driver.Chassis)
        reader, writer = os.pipe()
        self.addCleanup(os.close, reader)
        self.addCleanup(os.close, writer)
        records = []
        car.link, car.parser = NS(fd=reader), driver.chassis_core.Parser()
        car.last_rx, car.status = 3., {'ack': 17}
        car.observe = lambda event, **fields: records.append((event, fields))
        first = driver.chassis_core.packet(0x92, 65535, struct.pack('<15I', *REGISTER_VALUES))
        second_values = list(REGISTER_VALUES)
        second_values[4] = 999
        second = driver.chassis_core.packet(0x92, 0, struct.pack('<15I', *second_values))
        bad = first[:-1] + bytes([first[-1] ^ 1])
        os.write(writer, b'noise' + bad + first[:20])
        car.poll(0)
        self.assertEqual(records, [])
        os.write(writer, first[20:] + second)
        with patch.object(driver.c30d_link.time, 'monotonic', return_value=12.5):
            car.poll(0)
        self.assertEqual([event for event, fields in records], ['motor_registers']*2)
        self.assertEqual([fields['mcu_frame_seq'] for event, fields in records], [65535, 0])
        expected = dict(version=1, tim1_cr1=129, tim1_ccer=4369, tim1_bdtr=32768,
                        tim1_ccr1=1111, tim1_ccr2=222, tim1_ccr3=333, tim1_ccr4=444,
                        tim9_ccr1=555, tim9_ccr2=666, tim10_ccr1=777, tim11_ccr1=888,
                        gpiod_idr=8, gpiob_afr_high=819, gpioe_afr_high=51,
                        host_monotonic_s=12.5, mcu_frame_seq=65535)
        self.assertEqual(records[0][1], expected)
        self.assertEqual(records[1][1]['tim1_ccr1'], 999)
        self.assertEqual(car.last_rx, 3., 'register replies are not fresh 0x90 telemetry')
        self.assertEqual(car.status, {'ack': 17}, 'register replies do not accept a motor command')

    def test_motor_register_observation_defaults_off_and_mock_never_opens_serial(self):
        for config in ({}, self.speed_config()):
            with self.subTest(config=config):
                run = self.run_driver([(.125, 0)]*8, config=config)
                self.assert_clean(run)
                self.assertNotIn(0x15, [kind for kind, seq, payload in run.frames])
                self.assertFalse(any(event == 'motor_registers' for event, fields in run.records))
        run = self.run_driver([(.125, 0)]*8,
                              ros_params={'~mock': True, '~observe_motor_registers': True})
        self.assertIsNone(run.error)
        self.assertEqual(run.links, [])
        self.assertEqual(run.frames, [])

    def test_motor_register_observation_follows_nonzero_send_at_most_five_hz(self):
        for config, command_kind in (({}, 0x11), (self.speed_config(), 0x10)):
            with self.subTest(command_kind=command_kind):
                run = self.run_driver([(.125, 0)]*12 + [(0, 0)]*5, config=config,
                                      ros_params={'~observe_motor_registers': True})
                self.assert_clean(run)
                probes = [fields for event, fields in run.records
                          if event == 'serial_tx' and fields['kind'] == 0x15]
                self.assertGreaterEqual(len(probes), 2, 'opt-in must send register requests while nonzero')
                for before, after in zip(probes, probes[1:]):
                    self.assertGreaterEqual(after['tx_start_monotonic_s'] - before['tx_start_monotonic_s'],
                                            .2 - 1e-12)
                for i, (kind, seq, payload) in enumerate(run.frames):
                    if kind == 0x15:
                        previous_kind, previous_seq, previous_payload = run.frames[i-1]
                        self.assertEqual(previous_kind, command_kind)
                        speeds = struct.unpack('<4h', previous_payload) if command_kind == 0x10 else struct.unpack('<I4h', previous_payload)[1:]
                        self.assertTrue(any(speeds), 'zero and close must never schedule register requests')
                replies = [fields for event, fields in run.records if event == 'motor_registers']
                self.assertEqual(len(replies), len(probes))
                self.assertTrue(all(fields['tim1_ccr1'] == 1111 for fields in replies),
                                'publish actual reply registers, independently of software duty')
                self.assertTrue(all('mcu_frame_seq' in fields and 'host_monotonic_s' in fields
                                    for fields in replies))

    def test_missing_motor_register_reply_keeps_control_refresh_zero_and_close(self):
        run = self.run_driver([(.125, 0)]*12 + [(0, 0), None, None, None, None, None, None],
                              config=self.speed_config(), motor_register_reply=False,
                              ros_params={'~observe_motor_registers': True})
        self.assert_clean(run)
        self.assertEqual(run.output, [], 'observation must not enable a PWM fallback')
        self.assertEqual(run.speed_output[:12], [[125]*4]*12)
        self.assertTrue(all(row == [0]*4 for row in run.speed_output[12:]))
        tx = [fields for event, fields in run.records if event == 'command_tx']
        for before, after in zip(tx, tx[1:]):
            self.assertAlmostEqual(after['host_monotonic_s'] - before['host_monotonic_s'], .05)
        self.assertIn('command_timeout', [fields['stop_reason'] for fields in tx])
        self.assertGreaterEqual(sum(kind == 0x15 for kind, seq, payload in run.frames), 2)
        self.assertFalse(any(event == 'motor_registers' for event, fields in run.records))
        self.assertEqual(run.speed_output[-10:], [[0]*4]*10)

    def test_motor_register_request_or_diagnostic_error_does_not_block_zero_or_close(self):
        for options in (dict(fail_motor_register_request=True), dict(diagnostics_fail=True)):
            with self.subTest(options=options):
                run = self.run_driver([(.125, 0)]*8 + [(0, 0)], config=self.speed_config(),
                                      ros_params={'~observe_motor_registers': True}, **options)
                self.assert_clean(run)
                self.assertEqual(run.output, [])
                self.assertEqual(run.speed_output[:8], [[125]*4]*8)
                self.assertEqual(run.speed_output[-11:], [[0]*4]*11)
                self.assertGreaterEqual(sum(kind == 0x15 for kind, seq, payload in run.frames), 2)
                if options.get('fail_motor_register_request'):
                    self.assertTrue(any('TEST_ONLY motor register request write failure' in str(args)
                                        for args in run.warn_logs), 'request write failures need a warning')

    def test_motor_register_observation_preserves_fault_zero_before_diagnostic_and_close(self):
        run = self.run_driver([(.125, 0)]*3, config=self.speed_config(), flags=[56, 56, 60],
                              ros_params={'~observe_motor_registers': True})
        self.assertIsInstance(run.error, RuntimeError)
        self.assertIn('MCU fault', str(run.error))
        self.assertTrue(run.links[0].closed)
        fault_request = next(i for i, frame in enumerate(run.frames) if frame[0] == 0x14)
        self.assertIn(0x15, [kind for kind, seq, payload in run.frames[:fault_request]])
        self.assertEqual(run.frames[fault_request-1][0], 0x10)
        self.assertEqual(run.frames[fault_request-1][2], b'\0'*8)
        self.assertNotIn(0x15, [kind for kind, seq, payload in run.frames[fault_request:]])
        self.assertEqual(run.speed_output[-10:], [[0]*4]*10)
        self.assertEqual(run.output, [])

    def test_motor_register_frames_cannot_hide_a_telemetry_timeout(self):
        run = self.run_driver([(.125, 0)]*5, config=self.speed_config(), interval=.2,
                              telemetry=[True, False, False, False, False],
                              register_frames_each_loop=True,
                              ros_params={'~observe_motor_registers': True})
        self.assertIsInstance(run.error, RuntimeError)
        self.assertIn('telemetry timeout', str(run.error))
        self.assertTrue(run.links[0].closed)
        self.assertTrue(any(event == 'motor_registers' for event, fields in run.records))
        self.assertEqual(run.speed_output[:3], [[125]*4, [125]*4, [0]*4])
        self.assertTrue(all(row == [0]*4 for row in run.speed_output[2:]))


if __name__ == '__main__':
    unittest.main()
