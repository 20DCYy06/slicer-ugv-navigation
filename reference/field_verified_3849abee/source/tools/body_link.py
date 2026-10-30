"""SLC2车身扩展协议；复用生产驱动的USB身份核对、CRC和零速关闭。"""
import os
from pathlib import Path
import select
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'catkin_ws/src/Slicer_wasd_control/scripts'))
from c30d_link import Chassis, decode_fault_diagnostic, decode_motor_registers
from control_core import load_config

BODY_FORMAT = struct.Struct('<4BI4h4h3h3h3h3h4h4H')


def decode_body(payload):
    if len(payload) != 64 or payload[0] != 1:
        raise ValueError('Unsupported body telemetry layout/version')
    v = BODY_FORMAT.unpack(payload)
    return dict(version=v[0], imu_flags=v[1], who_am_i=v[2], source=v[3],
                mcu_us=v[4], delta_raw_abcd=list(v[5:9]), wheel_mm_s=list(v[9:13]),
                gyro_raw=list(v[13:16]), accel_raw=list(v[16:19]),
                gyro_rad_s=[x*.001 for x in v[19:22]], accel_m_s2=[x*.001 for x in v[22:25]],
                vx_m_s=v[25]*.001, wz_rad_s=v[26]*.001,
                target_vx_m_s=v[27]*.001, target_wz_rad_s=v[28]*.001,
                voltage=v[29]*.001, status_flags=v[30], imu_age_ms=v[31], ack_seq=v[32])


class BodyChassis(Chassis):
    def __init__(self, config):
        self.body_telemetry = None
        self.body_info = None
        super().__init__(config)

    def poll(self, timeout=0):
        if not select.select([self.link.fd], [], [], timeout)[0]:
            return
        raw = os.read(self.link.fd, 4096)
        if not raw:
            raise RuntimeError('Serial disconnected')
        for kind, seq, data in self.parser.feed(raw):
            if kind == 0x93:
                self.body_telemetry = decode_body(data)
            elif kind == 0x84 and len(data) == 8:
                self.body_info = struct.unpack('<IBBBB', data)
            elif kind == 0x81 and len(data) == 14:
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
                self.status = {'tick': v[0], 'interval_us': v[1], 'delta': v[2:6],
                               'flags': v[6], 'ack': v[7], 'raw': v[8:12],
                               'mm_s': v[12:16], 'pwm': v[16:20], 'voltage': v[20]/1000}
                self.last_rx = time.monotonic()

    def extension_info(self):
        self.send(0x17, struct.pack('<I', self.nonce))
        self.wait(lambda: self.body_info is not None, 1.)
        if self.body_info[0] != self.nonce:
            raise RuntimeError('Wrong body-info nonce')
        return self.body_info

    def body_speed(self, vx_m_s, wz_rad_s):
        vx, wz = round(vx_m_s*1000), round(wz_rad_s*1000)
        if not -32768 <= vx <= 32767 or not -32768 <= wz <= 32767:
            raise ValueError('Body setpoint exceeds signed 16-bit range')
        return self.send(0x16, struct.pack('<Ihh', self.nonce, vx, wz))

    def require_body_control(self, timeout=5.0):
        """确认扩展与IMU就绪，不把旧固件的轮速接口当作车身闭环。"""
        info = self.extension_info()
        if info[1] != 1 or info[2] != 0xea:
            raise RuntimeError('Body/IMU firmware extension not available: %r' % (info,))
        self.wait(lambda: self.body_telemetry is not None and
                  self.body_telemetry['imu_flags'] & 7 == 7, timeout)
        return info
