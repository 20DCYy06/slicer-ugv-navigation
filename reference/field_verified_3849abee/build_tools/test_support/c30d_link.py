#!/usr/bin/env python3
"""SLC2 serial transport. No ROS dependency; same CRC envelope as future ROS driver."""
import glob
import os
from pathlib import Path
import secrets
import select
import struct
import sys
import time

from serial_transport import Link, identity
from chassis_core import Parser, packet


class Chassis:
    def __init__(self, cfg):
        self.cfg = cfg
        self.seq = 0
        self.nonce = secrets.randbits(32)
        self.parser = Parser()
        self.status = None
        self.last_rx = 0.0
        self.reply = None
        self.configuration_ack = None
        self.test_capabilities = None
        self.fault_diagnostic = None
        self.motor_registers = None
        self.calibration_frames = 0
        self.link = None
        self.expected_counts = cfg['counts_per_revolution']
        self.max_wheel_mm_s = cfg['max_wheel_mm_s']
        if not 1 <= self.expected_counts <= 65535 or not 1 <= self.max_wheel_mm_s <= 800:
            raise ValueError('Invalid configured encoder resolution/wheel speed limit')
        expected = ('1a86', '55d4', str(cfg['serial_number']))
        if (cfg['usb_vid'], cfg['usb_pid'], cfg['baud']) != (0x1a86, 0x55d4, 115200):
            raise ValueError('C30D identity/baud must be 1a86:55d4 / 115200')
        paths = sorted(set(glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyCH*') +
                           glob.glob('/dev/ttyUSB*'))) if cfg['serial_port'] == 'auto' else [cfg['serial_port']]
        matched = []
        for p in paths:
            try:
                if identity(p) == expected:
                    matched.append(os.path.realpath(p))
            except OSError:
                continue
        matched = sorted(set(matched))
        if len(matched) != 1:
            raise RuntimeError('Expected unique C30D %s; found %s. No RTK fallback.' %
                               (expected, matched))
        self.device = matched[0]
        # Dedicated MCU port: never restore inherited 9600/HUPCL on exit.
        # Do not toggle modem/reset lines; legacy tools keep default restoration.
        self.link = Link(self.device, expected, restore_on_close=False)
        try:
            self.send(1, struct.pack('<I', self.nonce))
            self.wait(lambda: self.reply is not None, 2.0)
            values = struct.unpack('<4sBBHHI', self.reply)
            if values != (b'SLC2', 2, 4, self.expected_counts, 500, self.nonce):
                raise RuntimeError('Wrong firmware/nonce: %r' % (values,))
            self.wait(lambda: self.status is not None, 1.0)
            print('[CONNECTED] %s SLC2, %d counts/rev (provisional), MCU watchdog=500ms' %
                  (self.device, self.expected_counts), flush=True)
        except BaseException:
            self.close()
            raise

    def send(self, kind, payload):
        self.seq = (self.seq + 1) & 65535
        self.link.send(packet(kind, self.seq, payload))
        return self.seq

    def poll(self, timeout=0):
        if select.select([self.link.fd], [], [], timeout)[0]:
            raw = os.read(self.link.fd, 4096)
            if not raw:
                raise RuntimeError('Serial disconnected')
            for kind, seq, data in self.parser.feed(raw):
                if kind == 0x81 and len(data) == 14:
                    self.reply = data
                elif kind == 0x82 and len(data) == 5:
                    self.configuration_ack = struct.unpack('<IB', data)
                elif kind == 0x83 and len(data) == 9:
                    self.test_capabilities = struct.unpack('<IBHH', data)
                elif kind == 0x91 and len(data) == 42:
                    self.fault_diagnostic = decode_fault_diagnostic(data)
                elif kind == 0x92 and len(data) == 60:
                    self.motor_registers = decode_motor_registers(data)
                elif kind == 0x90 and len(data) == 61:
                    v = struct.unpack('<II4iBH4i4h4hH', data)
                    if v[6] & 64:
                        self.calibration_frames += 1
                    self.status = {'tick': v[0], 'interval_us': v[1], 'delta': v[2:6],
                                   'flags': v[6], 'ack': v[7], 'raw': v[8:12],
                                   'mm_s': v[12:16], 'pwm': v[16:20], 'voltage': v[20] / 1000}
                    self.last_rx = time.monotonic()

    def wait(self, predicate, timeout):
        end = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= end:
                raise TimeoutError('MCU reply timeout')
            self.poll(.02)

    def speed(self, mm_s):
        if len(mm_s) != 4 or any(abs(x) > self.max_wheel_mm_s for x in mm_s):
            raise ValueError('Wheel speed limit +/-%d mm/s' % self.max_wheel_mm_s)
        return self.send(0x10, struct.pack('<4h', *mm_s))

    def require_pwm_test(self):
        self.test_capabilities = None
        self.send(0x13, struct.pack('<I', self.nonce))
        try:
            self.wait(lambda: self.test_capabilities is not None, 1)
        except TimeoutError as exc:
            raise RuntimeError('Firmware has no PWM test capability; install updated firmware first') from exc
        nonce, supported, cap, duration = self.test_capabilities
        if nonce != self.nonce or supported != 1 or not 1 <= cap <= 750 or duration != 0:
            raise RuntimeError('Unexpected PWM test capability')
        return cap, duration

    def pwm(self, permille):
        limit = self.cfg.get('test_pwm_limit_permille', 250)
        if len(permille) != 4 or any(abs(x) > limit for x in permille):
            raise ValueError('PWM test limited to +/-%s permille' % limit)
        return self.send(0x11, struct.pack('<I4h', self.nonce, *permille))

    def configure(self, signs):
        pwm = self.cfg['pwm_signs']
        if len(pwm) != 4 or len(signs) != 4 or any(x not in (-1, 1) for x in pwm + list(signs)):
            raise ValueError('Four +/-1 polarity values required')
        self.configuration_ack = None
        self.send(0x12, struct.pack('<I8b', self.nonce, *(pwm + list(signs))))
        self.wait(lambda: self.configuration_ack == (self.nonce, 1), 1)
        self.wait(lambda: self.status is not None and self.status['flags'] & 8, 1)

    def read_fault_diagnostic(self, timeout=.3):
        """Read-only request; old firmware times out without changing outputs."""
        self.fault_diagnostic = None
        self.send(0x14, struct.pack('<I', self.nonce))
        self.wait(lambda: self.fault_diagnostic is not None, timeout)
        return self.fault_diagnostic

    def read_motor_registers(self, timeout=.3):
        """Read-only snapshot of the actual timer/GPIO registers during motion."""
        self.motor_registers = None
        self.send(0x15, struct.pack('<I', self.nonce))
        self.wait(lambda: self.motor_registers is not None, timeout)
        return self.motor_registers

    def line(self):
        s = self.status
        return 'V=%.2f EN=%d power=%d configured=%d fault=%d counts=%s mm/s=%s PWM=%s' % (
            s['voltage'], bool(s['flags'] & 16), bool(s['flags'] & 32),
            bool(s['flags'] & 8), bool(s['flags'] & 4), s['raw'], s['mm_s'], s['pwm'])

    def close(self):
        if self.link is not None:
            try:
                for _ in range(10):
                    self.speed([0] * 4)
                    time.sleep(.05)
            except Exception as exc:
                print('[STOP FAILED] Cut motor power:', exc, flush=True)
            finally:
                self.link.close()
                self.link = None


def decode_fault_diagnostic(data):
    """0x91 v1: first fault before stop; later fault bits remain visible."""
    v = struct.unpack('<4BIII4h4h4h2B', data)
    if v[0] != 1:
        raise ValueError('Unsupported MCU fault diagnostic version')
    sources = {0: 'none', 1: 'uart_rx_error', 2: 'uart_rx_overflow',
               3: 'uart_tx_timeout', 4: 'encoder_delta_limit',
               5: 'closed_loop_stall_or_direction', 6: 'control_loop_late'}
    labels = {1: 'UART', 2: 'ENCODER_DELTA', 4: 'PID_STALL_OR_DIRECTION',
              8: 'CONTROL_TIMING'}
    return {'version': v[0], 'fault_mask': v[1], 'first_fault_mask': v[2],
            'fault_names': [name for bit, name in labels.items() if v[1] & bit],
            'source': sources.get(v[3], 'unknown_%d' % v[3]),
            'mcu_ms': v[4], 'control_interval_ms': v[5], 'uart_sr': v[6],
            'encoder_delta_abcd': list(v[7:11]),
            'command_abcd': list(v[11:15]), 'applied_pwm_ticks_abcd': list(v[15:19]),
            'mode': {0: 'idle', 1: 'pwm_test', 2: 'closed_loop', 3: 'calibration'}.get(v[19], 'unknown'),
            'command_unit': {0: 'unused', 1: 'permille', 2: 'mm/s', 3: 'unused'}.get(v[19], 'unknown'),
            'motor_enabled': bool(v[20] & 1), 'motor_power_present': bool(v[20] & 2)}


def decode_motor_registers(data):
    names = ('version', 'tim1_cr1', 'tim1_ccer', 'tim1_bdtr',
             'tim1_ccr1', 'tim1_ccr2', 'tim1_ccr3', 'tim1_ccr4',
             'tim9_ccr1', 'tim9_ccr2', 'tim10_ccr1', 'tim11_ccr1',
             'gpiod_idr', 'gpiob_afr_high', 'gpioe_afr_high')
    values = struct.unpack('<15I', data)
    if values[0] != 1:
        raise ValueError('Unsupported motor register diagnostic version')
    return dict(zip(names, values))
