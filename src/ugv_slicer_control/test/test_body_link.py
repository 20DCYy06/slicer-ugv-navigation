#!/usr/bin/env python3
"""TEST_ONLY: actual Chassis/codec with anonymous pipes and no devices or ROS."""
import binascii
from contextlib import ExitStack
import importlib.util
import math
import os
from pathlib import Path
import struct
import sys
import unittest
from unittest.mock import patch


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
CONFIG = dict(serial_port='/TEST_ONLY_BODY', usb_vid=0x1a86, usb_pid=0x55d4,
              serial_number='TEST_ONLY', baud=115200, counts_per_revolution=60000,
              max_wheel_mm_s=600, control_mode='body_speed')
BODY_FORMAT = '<4BI4h4h3h3h3h3h4h4H'


def body_payload(tick=100000, flags=7, who=0xea, age=10, version=1):
    return struct.pack(BODY_FORMAT, version, flags, who, 3, tick,
                       -1, 2, -3, 4, -100, 200, -300, 400,
                       -11, 22, -33, 44, -55, 66,
                       -1000, 2000, -3000, 9810, -2000, 3000,
                       -125, 750, 250, -500, 12500, 15, age, 321)


class BodyLinkTests(unittest.TestCase):
    def chassis(self, info=(1, 0xea, 7, 3), telemetry=None, wrong_nonce=False,
                observe=None, mode='body_speed', fail_zero=False, bad_hello=False):
        stack = ExitStack()
        self.addCleanup(stack.close)
        old_path = list(sys.path)
        stack.callback(lambda: sys.path.__setitem__(slice(None), old_path))
        sys.path.insert(0, str(SCRIPTS))
        spec = importlib.util.spec_from_file_location('body_link_under_test', SCRIPTS / 'c30d_link.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        clock, links = [10.], []

        class PipeLink:
            def __init__(inner, device, expected, restore_on_close=True):
                if (device, expected, restore_on_close) != (
                        '/TEST_ONLY_BODY', ('1a86', '55d4', 'TEST_ONLY'), False):
                    raise AssertionError('identity and dedicated-port boundary changed')
                inner.fd, inner.writer = os.pipe()
                inner.sent, inner.raw, inner.closed = [], [], False
                inner.parser = module.Parser()
                links.append(inner)

            def emit(inner, kind, payload, seq=123):
                os.write(inner.writer, module.packet(kind, seq, payload))

            def status(inner, tick=100000):
                inner.emit(0x90, struct.pack('<II4iBH4i4h4hH', tick, 50000,
                                             *([0]*4), 56, 0, *([0]*12), 12500))

            def send(inner, raw):
                frames = inner.parser.feed(raw)
                if len(frames) != 1:
                    raise AssertionError('Chassis must send one valid CRC frame')
                kind, seq, payload = frames[0]
                inner.raw.append(raw)
                inner.sent.append((kind, seq, payload))
                if kind == 1:
                    inner.nonce, = struct.unpack('<I', payload)
                    inner.emit(0x81, struct.pack('<4sBBHHI', b'SLC2', 3 if bad_hello else 2,
                                                4, 60000, 500, inner.nonce))
                    inner.status()
                elif kind == 0x17 and info is not None:
                    inner.emit(0x84, struct.pack('<IBBBB', inner.nonce ^ int(wrong_nonce), *info))
                    if telemetry is not None:
                        inner.emit(0x93, telemetry)
                elif kind == 0x16 and fail_zero:
                    raise TimeoutError('TEST_ONLY serial failure')

            def close(inner):
                if not inner.closed:
                    inner.closed = True
                    os.close(inner.fd)
                    os.close(inner.writer)

        stack.enter_context(patch.object(module, 'identity', return_value=('1a86', '55d4', 'TEST_ONLY')))
        stack.enter_context(patch.object(module, 'Link', PipeLink))
        stack.enter_context(patch.object(module.secrets, 'randbits', return_value=0x12345678))
        stack.enter_context(patch.object(module.time, 'monotonic', side_effect=lambda: clock[0]))
        stack.enter_context(patch.object(module.time, 'sleep', side_effect=lambda delay: clock.__setitem__(0, clock[0]+delay)))
        # Move the test clock on an empty pipe so production wait() can time out.
        real_poll = module.Chassis.poll

        def poll(chassis, timeout=0):
            real_poll(chassis, 0)
            clock[0] += timeout

        stack.enter_context(patch.object(module.Chassis, 'poll', poll))
        if bad_hello:
            with self.assertRaises(RuntimeError):
                module.Chassis(dict(CONFIG, control_mode=mode), observe=observe)
            return module, None, links[0], clock
        chassis = module.Chassis(dict(CONFIG, control_mode=mode), observe=observe)
        self.addCleanup(chassis.close)
        return module, chassis, links[0], clock

    def require_api(self, chassis, *names):
        for name in names:
            self.assertTrue(hasattr(chassis, name), 'missing body protocol API: '+name)

    def test_decode_layout_has_reference_names_and_si_units(self):
        module, _, _, _ = self.chassis()
        self.require_api(module, 'decode_body')
        decoded = module.decode_body(body_payload())
        self.assertEqual(decoded, dict(version=1, imu_flags=7, who_am_i=0xea, source=3,
                         mcu_us=100000, delta_raw_abcd=[-1, 2, -3, 4],
                         wheel_mm_s=[-100, 200, -300, 400], gyro_raw=[-11, 22, -33],
                         accel_raw=[44, -55, 66], gyro_rad_s=[-1., 2., -3.],
                         accel_m_s2=[9.81, -2., 3.], vx_m_s=-.125, wz_rad_s=.750,
                         target_vx_m_s=.250, target_wz_rad_s=-.500,
                         voltage=12.5, status_flags=15, imu_age_ms=10, ack_seq=321))

    def test_decode_rejects_wrong_version_and_exact_length(self):
        module, _, _, _ = self.chassis()
        self.require_api(module, 'decode_body')
        for payload in (b'', body_payload()[:-1], body_payload()+b'\x00', body_payload(version=2)):
            with self.subTest(payload_length=len(payload)), self.assertRaises(ValueError):
                module.decode_body(payload)

    def test_body_speed_emits_golden_si_frame_and_diagnostics(self):
        events = []
        _, chassis, link, _ = self.chassis(observe=lambda event, **fields: events.append((event, fields)))
        self.require_api(chassis, 'body_speed')
        seq = chassis.body_speed(.1234, -.4567)
        self.assertEqual(seq, 2)
        # Header/version/kind/length/sequence and payload derived independently.
        body = bytes.fromhex('0116080200785634127b0037fe')
        expected = b'\xa5\x5a'+body+struct.pack('<H', binascii.crc_hqx(body, 0xffff))
        self.assertEqual(link.raw[-1], expected)
        event, fields = events[-1]
        self.assertEqual(event, 'serial_tx')
        self.assertEqual((fields['kind'], fields['command_seq'], fields['body_vx_mm_s'],
                          fields['body_wz_mrad_s'], fields['success']), (22, 2, 123, -457, True))

    def test_body_speed_rejects_nonfinite_and_overflow_without_write(self):
        _, chassis, link, _ = self.chassis()
        self.require_api(chassis, 'body_speed')
        count = len(link.sent)
        for vx, wz in ((math.nan, 0), (0, math.inf), (-math.inf, 0),
                       (32.768, 0), (0, -32.769), (1e308, 0)):
            with self.subTest(vx=vx, wz=wz), self.assertRaises(ValueError):
                chassis.body_speed(vx, wz)
        self.assertEqual(len(link.sent), count)
        chassis.body_speed(-32.768, 32.767)
        self.assertEqual(struct.unpack('<Ihh', link.sent[-1][2]), (0x12345678, -32768, 32767))

    def test_extension_query_clears_cached_reply_and_matches_session_nonce(self):
        events = []
        _, chassis, link, _ = self.chassis(observe=lambda event, **fields: events.append((event, fields)))
        self.require_api(chassis, 'extension_info', 'body_info')
        chassis.body_info = (0, 0, 0, 0, 0)
        self.assertEqual(chassis.extension_info(), (0x12345678, 1, 0xea, 7, 3))
        self.assertEqual(link.sent[-1][0], 0x17)
        self.assertEqual(link.sent[-1][2], b'\x78\x56\x34\x12')
        body_events = [fields for event, fields in events if event == 'body_info']
        self.assertEqual(len(body_events), 1)
        self.assertEqual(body_events[0]['mcu_frame_seq'], 123)

    def test_missing_extension_cannot_use_cached_info(self):
        _, chassis, _, _ = self.chassis(info=None)
        self.require_api(chassis, 'extension_info', 'body_info')
        chassis.body_info = (0x12345678, 1, 0xea, 7, 3)
        with self.assertRaises((RuntimeError, TimeoutError)):
            chassis.extension_info()

    def test_wrong_nonce_is_rejected(self):
        _, chassis, _, _ = self.chassis(wrong_nonce=True)
        self.require_api(chassis, 'extension_info')
        with self.assertRaises(RuntimeError):
            chassis.extension_info()

    def test_require_body_control_waits_for_ready_fresh_telemetry(self):
        _, chassis, _, _ = self.chassis(info=(1, 0xea, 3, 0), telemetry=body_payload())
        self.require_api(chassis, 'require_body_control', 'body_feedback_fresh')
        self.assertEqual(chassis.require_body_control(), (0x12345678, 1, 0xea, 3, 0))
        self.assertTrue(chassis.body_feedback_fresh())

    def test_require_body_control_rejects_unsupported_imu(self):
        for info in ((2, 0xea, 7, 3), (1, 0, 7, 3), (1, 0xea, 15, 3)):
            with self.subTest(info=info):
                _, chassis, _, _ = self.chassis(info=info, telemetry=body_payload())
                self.require_api(chassis, 'require_body_control')
                with self.assertRaises(RuntimeError):
                    chassis.require_body_control(timeout=.04)

    def test_require_body_control_rejects_unready_or_aged_feedback(self):
        for payload in (body_payload(flags=3), body_payload(flags=15),
                        body_payload(who=0), body_payload(age=101)):
            with self.subTest(payload=payload.hex()):
                _, chassis, _, _ = self.chassis(telemetry=payload)
                self.require_api(chassis, 'require_body_control')
                with self.assertRaises((RuntimeError, TimeoutError)):
                    chassis.require_body_control(timeout=.04)

    def test_poll_preserves_reference_fields_and_ordinary_rx_clock(self):
        events = []
        _, chassis, link, clock = self.chassis(observe=lambda event, **fields: events.append((event, fields)))
        self.require_api(chassis, 'body_telemetry', 'last_body_rx', 'body_feedback_fresh')
        self.assertFalse(chassis.body_feedback_fresh())
        ordinary_rx = chassis.last_rx
        clock[0] += .1
        link.emit(0x93, body_payload(), seq=65534)
        chassis.poll()
        self.assertEqual(chassis.last_rx, ordinary_rx)
        self.assertEqual(chassis.last_body_rx, clock[0])
        fields, = [fields for event, fields in events if event == 'body_telemetry']
        self.assertEqual(fields['mcu_frame_seq'], 65534)
        self.assertEqual(fields['mcu_us'], 100000)
        self.assertEqual(fields['wheel_mm_s'], [-100, 200, -300, 400])
        self.assertEqual(fields['gyro_rad_s'], [-1., 2., -3.])
        self.assertEqual(fields['target_wz_rad_s'], -.5)
        self.assertTrue(chassis.body_feedback_fresh())

    def test_ordinary_telemetry_and_duplicate_ticks_cannot_keep_body_alive(self):
        _, chassis, link, clock = self.chassis()
        self.require_api(chassis, 'body_feedback_fresh', 'last_body_rx')
        link.emit(0x93, body_payload())
        chassis.poll()
        initial_rx = chassis.last_body_rx
        for dt in (.15, .16):
            clock[0] += dt
            link.status()
            link.emit(0x93, body_payload())
            chassis.poll()
        self.assertEqual(chassis.last_rx, clock[0])
        self.assertEqual(chassis.last_body_rx, initial_rx)
        self.assertFalse(chassis.body_feedback_fresh())

    def test_tick_wrap_advances_but_backward_tick_does_not(self):
        _, chassis, link, clock = self.chassis()
        self.require_api(chassis, 'body_feedback_fresh', 'last_body_rx')
        link.emit(0x93, body_payload(tick=0xfffffff0))
        chassis.poll()
        clock[0] += .2
        link.emit(0x93, body_payload(tick=0x20))
        chassis.poll()
        wrapped_rx = chassis.last_body_rx
        self.assertEqual(wrapped_rx, clock[0])
        self.assertTrue(chassis.body_feedback_fresh())
        clock[0] += .2
        link.emit(0x93, body_payload(tick=0x10))
        chassis.poll()
        self.assertEqual(chassis.last_body_rx, wrapped_rx)
        clock[0] += .11
        self.assertFalse(chassis.body_feedback_fresh())

    def test_failed_flags_and_imu_age_invalidate_feedback_immediately(self):
        _, chassis, link, _ = self.chassis()
        self.require_api(chassis, 'body_feedback_fresh')
        for tick, flags, who, age, fresh in ((1, 7, 0xea, 100, True), (2, 15, 0xea, 0, False),
                                           (3, 3, 0xea, 0, False), (4, 7, 0, 0, False),
                                           (5, 7, 0xea, 101, False)):
            link.emit(0x93, body_payload(tick=tick, flags=flags, who=who, age=age))
            chassis.poll()
            self.assertEqual(chassis.body_feedback_fresh(), fresh)

    def test_close_body_mode_sends_ten_body_zeros_despite_observer_errors(self):
        def failed_observer(event, **fields):
            raise RuntimeError('TEST_ONLY diagnostics unavailable')
        _, chassis, link, _ = self.chassis(observe=failed_observer)
        self.require_api(chassis, 'body_speed')
        chassis.close()
        self.assertEqual([kind for kind, _, _ in link.sent[1:]], [0x16]*10)
        self.assertEqual([struct.unpack('<Ihh', payload) for _, _, payload in link.sent[1:]],
                         [(0x12345678, 0, 0)]*10)
        self.assertTrue(link.closed)
        self.assertIsNone(chassis.link)

    def test_body_close_serial_error_still_closes_without_legacy_fallback(self):
        _, chassis, link, _ = self.chassis(fail_zero=True)
        self.require_api(chassis, 'body_speed')
        chassis.close()
        self.assertEqual([kind for kind, _, _ in link.sent[1:]], [0x16])
        self.assertTrue(link.closed)
        self.assertIsNone(chassis.link)

    def test_constructor_failure_closes_in_configured_body_mode(self):
        _, _, link, _ = self.chassis(bad_hello=True)
        self.assertEqual([kind for kind, _, _ in link.sent[1:]], [0x16]*10)
        self.assertTrue(link.closed)

    def test_default_close_keeps_existing_wheel_speed_mode(self):
        _, chassis, link, _ = self.chassis(mode='pwm_test')
        chassis.close()
        self.assertEqual([kind for kind, _, _ in link.sent[1:]], [0x10]*10)
        self.assertTrue(all(payload == b'\x00'*8 for _, _, payload in link.sent[1:]))


if __name__ == '__main__':
    unittest.main()
