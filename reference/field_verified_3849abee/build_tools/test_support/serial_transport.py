#!/usr/bin/env python3
"""C30D 原厂 7B/7D 协议的独立、限时串口测试；不修改 ROS 驱动。

默认只接收。--power-off-test 仅供动力电断开时测试发送路径，不能证明
下位机已接受指令。原厂固件的掉线停车功能尚未验证，禁止据此直接试车。
参考报文实现：
https://github.com/ericlai0323/turn_on_wheeltec_robot/blob/master/src/wheeltec_robot.cpp
"""
import argparse
import fcntl
import glob
import math
import os
from pathlib import Path
import select
import signal
import struct
import termios
import time


def identity(device):
    stat = os.stat(device)
    path = Path('/sys/dev/char/%d:%d/device' %
                (os.major(stat.st_rdev), os.minor(stat.st_rdev))).resolve()
    for parent in (path,) + tuple(path.parents):
        if (parent / 'idVendor').exists():
            return tuple((parent / name).read_text().strip() for name in
                         ('idVendor', 'idProduct', 'serial'))
    return None


def command(vx=0.0, wz=0.0):
    if not all(math.isfinite(x) for x in (vx, wz)):
        raise ValueError('Nonfinite speed')
    data = b'\x7b\x00\x00' + struct.pack('>hhh', round(vx * 1000), 0,
                                            round(wz * 1000))
    check = 0
    for byte in data:
        check ^= byte
    return data + bytes((check, 0x7d))


class Link:
    def __init__(self, device, expected, *, restore_on_close=True):
        self.fd = None
        self.old_termios2 = None
        self.restore_on_close = restore_on_close
        self.configured = False
        self.buffer = bytearray()
        self.frames = []
        self.tx = 0
        if identity(device) != expected:
            raise RuntimeError('USB identity mismatch; refusing to open')
        self.fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            # Verify the opened descriptor rather than trusting a mutable alias.
            if os.fstat(self.fd).st_rdev != os.stat(device).st_rdev:
                raise RuntimeError('Device changed while opening')
            if identity(device) != expected:
                raise RuntimeError('USB identity changed')
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            # Also exclude a second copy of this tool already holding the port.
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            # One atomic TCSETS2: no intermediate inherited 9600/8E1 setting.
            # Keep the complete old struct: tcgetattr's BOTHER value alone
            # cannot preserve the numeric custom baud for legacy restoration.
            buf = bytearray(44)
            fcntl.ioctl(self.fd, 0x802c542a, buf, True)  # TCGETS2
            self.old_termios2 = bytes(buf)
            fields = list(struct.unpack('=IIIIB19BII', buf))
            fields[0] = fields[1] = fields[3] = 0
            fields[2] = ((fields[2] & ~(termios.CSIZE | termios.PARENB |
                          termios.PARODD | termios.CSTOPB | getattr(termios, 'CRTSCTS', 0) |
                          termios.HUPCL | 0x100f | getattr(termios, 'CIBAUD', 0x100f0000))) |
                         termios.CS8 | termios.CREAD | termios.CLOCAL | 0x1000)
            fields[5 + termios.VMIN] = fields[5 + termios.VTIME] = 0
            fields[-2] = fields[-1] = 115200
            fcntl.ioctl(self.fd, 0x402c542b,
                        struct.pack('=IIIIB19BII', *fields))  # TCSETS2
            termios.tcflush(self.fd, termios.TCIFLUSH)
            self.configured = True
        except BaseException:
            self.close()
            raise

    def receive(self, timeout):
        if select.select([self.fd], [], [], timeout)[0]:
            data = os.read(self.fd, 4096)
            if not data:
                raise RuntimeError('Serial disconnected')
            self.buffer.extend(data)
        while len(self.buffer) >= 24:
            if self.buffer[0] != 0x7b:
                del self.buffer[0]
                continue
            frame = bytes(self.buffer[:24])
            check = 0
            for byte in frame[:22]:
                check ^= byte
            if frame[-1] == 0x7d and frame[22] == check:
                self.frames.append(frame)
                del self.buffer[:24]
            else:
                del self.buffer[0]

    def send(self, frame):
        remaining = memoryview(frame)
        deadline = time.monotonic() + 0.2
        while remaining:
            if time.monotonic() >= deadline:
                raise TimeoutError('Serial write timeout')
            if select.select([], [self.fd], [], 0.02)[1]:
                try:
                    count = os.write(self.fd, remaining)
                except BlockingIOError:
                    continue
                self.tx += count
                remaining = remaining[count:]

    def stream(self, frame, duration):
        end = time.monotonic() + duration
        next_tx = time.monotonic()
        while time.monotonic() < end:
            if time.monotonic() >= next_tx:
                self.send(frame)
                next_tx = time.monotonic() + 0.05  # 20 Hz
            self.receive(min(0.01, max(0.0, end - time.monotonic())))

    def close(self):
        if self.fd is not None:
            try:
                if self.old_termios2 is not None and (self.restore_on_close or not self.configured):
                    fcntl.ioctl(self.fd, 0x402c542b, self.old_termios2)
            finally:
                os.close(self.fd)
                self.fd = None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', help='Optional path; USB identity still checked')
    parser.add_argument('--serial-number', default='5CF0016377')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--power-off-test', action='store_true',
                      help='CONFIRM motor power OFF: zero, 0.02m/s for 0.3s, zero')
    mode.add_argument('--stop', action='store_true', help='Send zero only')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        assert command().hex() == '7b00000000000000007b7d'
        assert command(0.02).hex() == '7b00000014000000006f7d'
        assert command(-0.02)[3:5] == b'\xff\xec'
        print('Packet self-test PASS')
        return
    expected = ('1a86', '55d4', args.serial_number)
    paths = [args.device] if args.device else sorted(set(
        glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*') +
        glob.glob('/dev/ttyCH*')))
    matches = [p for p in paths if identity(p) == expected]
    if len(matches) != 1:
        raise RuntimeError('Expected exactly one C30D %s; found %s' %
                           (expected, matches))
    print('C30D:', matches[0], expected, '115200 8N1', flush=True)
    link = Link(matches[0], expected)
    sending = args.power_off_test or args.stop

    def interrupted(signum, frame):
        raise KeyboardInterrupt('Signal %s' % signum)

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    try:
        start = time.monotonic()
        while time.monotonic() - start < 1.5:
            link.receive(0.05)
        print('RX valid frames=%d (~%.1fHz)' %
              (len(link.frames), len(link.frames) / (time.monotonic() - start)),
              flush=True)
        if link.frames:
            print('RX sample:', link.frames[-1].hex(' '), flush=True)
        if args.power_off_test and len(link.frames) < 10:
            raise RuntimeError('Insufficient valid telemetry; nonzero test refused')
        if sending:
            print('TX zero:', command().hex(' '), flush=True)
            link.stream(command(), 0.5)
        if args.power_off_test:
            print('TX vx=0.020m/s, vy=wz=0, duration=0.30s:',
                  command(0.02).hex(' '), flush=True)
            link.stream(command(0.02), 0.3)
    finally:
        if sending:
            # Best effort only: cannot guarantee stopping after USB/host failure.
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            try:
                print('TX final zero x10', flush=True)
                for _ in range(10):
                    link.send(command())
                    time.sleep(0.05)
            except Exception as exc:
                print('STOP WRITE FAILED: disconnect motor power:', exc, flush=True)
                raise
            finally:
                link.close()
        else:
            link.close()
        print('Serial closed; total TX bytes=%d, valid RX frames=%d' %
              (link.tx, len(link.frames)), flush=True)
    print('Completed. TX success is NOT an MCU ACK or motor-response verification.')


if __name__ == '__main__':
    main()
