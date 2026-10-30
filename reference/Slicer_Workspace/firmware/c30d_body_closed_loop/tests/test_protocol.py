import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from body_link import BODY_FORMAT, BodyChassis, decode_body


class ProtocolTests(unittest.TestCase):
    def test_offsets_and_si_units(self):
        p = bytearray(64)
        struct.pack_into('<4BI', p, 0, 1, 7, 0xea, 3, 123456)
        struct.pack_into('<4h', p, 8, -1, 2, -3, 4)
        struct.pack_into('<3h', p, 36, 100, -200, 300)
        struct.pack_into('<3h', p, 42, 0, 0, 9807)
        struct.pack_into('<4h4H', p, 48, 125, 1000, 200, 1200, 12000, 15, 10, 99)
        t = decode_body(p)
        self.assertEqual(BODY_FORMAT.size, 64)
        self.assertEqual(t['delta_raw_abcd'], [-1, 2, -3, 4])
        self.assertAlmostEqual(t['wz_rad_s'], 1.)
        self.assertAlmostEqual(t['accel_m_s2'][2], 9.807)
        self.assertEqual(t['ack_seq'], 99)

    def test_body_command_frame_payload(self):
        car = object.__new__(BodyChassis)
        car.nonce = 123
        sent = []
        car.send = lambda kind, p: sent.append((kind, p))
        car.body_speed(.125, -1.2)
        self.assertEqual(sent, [(0x16, struct.pack('<Ihh', 123, 125, -1200))])


    def test_output_percent_cap_and_payload(self):
        for percent in range(101):
            car=object.__new__(BodyChassis);car.nonce=123;sent=[]
            def send(kind,payload):
                sent.append((kind,payload));car.output_limit_reply=(123,percent,800,1600,1280*percent//100)
            car.send=send
            car.wait=lambda predicate,timeout:self.assertTrue(predicate())
            applied=car.set_output_limit_percent(percent)
            self.assertEqual(sent,[(0x18,struct.pack('<IB',123,percent))])
            self.assertEqual(applied['applied_ticks'],1280*percent//100)
        for value in [-1,101,50.5,'100',True]:
            with self.assertRaises(ValueError):car.set_output_limit_percent(value)

    def test_output_percent_wrong_firmware_ack_rejected(self):
        car=object.__new__(BodyChassis);car.nonce=123
        car.send=lambda kind,payload:None
        for reply in [(124,100,800,1600,1280),(123,100,600,1600,960),(123,100,800,1600,1281)]:
            def wait(predicate,timeout):car.output_limit_reply=reply
            car.wait=wait
            with self.assertRaises(RuntimeError):car.set_output_limit_percent(100)


if __name__ == '__main__':
    unittest.main()
