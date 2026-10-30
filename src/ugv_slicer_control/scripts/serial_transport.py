#!/usr/bin/env python3
"""C30D USB identity and exclusive 115200 transport; no standalone CLI.

Derived from the recorded teammate serial_transport.py; see SOURCES.json.
"""
import fcntl
import os
from pathlib import Path
import select
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


    def close(self):
        if self.fd is not None:
            try:
                if self.old_termios2 is not None and (self.restore_on_close or not self.configured):
                    fcntl.ioctl(self.fd, 0x402c542b, self.old_termios2)
            finally:
                os.close(self.fd)
                self.fd = None


