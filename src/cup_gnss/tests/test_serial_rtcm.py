"""Synthetic single-owner serial queue checks; no USB/network acceptance claim."""
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from cup_gnss.sources import SerialSource, SourceProtocolError
from cup_gnss.ntrip import ForwardingBlocked, FreshGgaTracker, crc24q


def gga(quality=4, wall=None):
    epoch = time.strftime("%H%M%S", time.gmtime(time.time() if wall is None else wall))
    body = "GNGGA,{}.00,3100.000,N,11800.000,E,{},20,0.7,18,M,2,M,1,0001".format(epoch, quality)
    checksum = 0
    for byte in body.encode("ascii"):
        checksum ^= byte
    return "$" + body + "*{:02X}".format(checksum)


def frame():
    prefix = bytes((0xD3, 0, 2, 1005 >> 4, (1005 & 15) << 4))
    return prefix + crc24q(prefix).to_bytes(3, "big")


class Receiver:
    def __init__(self, short=False, error=False):
        self.calls = []
        self.short, self.error = short, error

    def write(self, payload):
        self.calls.append((threading.get_ident(), payload))
        if self.error:
            raise OSError("offline synthetic failure")
        return len(payload) - int(self.short)


class SerialRtcmCase(unittest.TestCase):
    def setUp(self):
        self.source = SerialSource(queue.Queue(), port="OFFLINE_FIXTURE")
        now, wall = time.monotonic(), time.time()
        for offset in (-1, 0):
            self.source.current_gga = {"raw": gga(wall=wall + offset), "received_mono": now + offset, "session": "session-a"}
            self.source.write_gga_tracker.update(self.source.current_gga, now_mono=now + offset, now_wall=wall + offset)
        self.receiver = Receiver()

    def request(self, **changes):
        result = {"frame": frame(), "session": "session-a", "deadline": time.monotonic() + 0.8,
                  "done": threading.Event(), "cancelled": False, "result": None, "error": None}
        result.update(changes)
        self.source.write_requests.put_nowait(result)
        return result

    def test_network_caller_never_holds_uart_and_gets_exact_owner_ack(self):
        results, errors = [], []
        def request_write():
            try:
                results.append(self.source.write_rtcm(frame()))
            except Exception as error:
                errors.append(error)
        caller = threading.Thread(target=request_write)
        caller.start()
        deadline = time.monotonic() + 0.5
        while self.source.write_requests.empty() and time.monotonic() < deadline:
            time.sleep(0.001)
        self.assertFalse(self.source.write_requests.empty())
        owner = threading.get_ident()
        self.source._service_writes(self.receiver, "session-a")
        caller.join(0.5)
        self.assertFalse(caller.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(results, [len(frame())])
        self.assertEqual(self.receiver.calls, [(owner, frame())])

    def test_expired_cancelled_and_old_session_never_write(self):
        for changes in ({"deadline": time.monotonic() - 1}, {"cancelled": True}, {"session": "old-session"}):
            with self.subTest(changes=changes):
                request = self.request(**changes)
                self.source._service_writes(self.receiver, "session-a")
                self.assertTrue(request["done"].is_set())
                self.assertIsNotNone(request["error"])
        self.assertEqual(self.receiver.calls, [])

    def test_pause_and_stale_or_no_fix_gga_never_write(self):
        with tempfile.TemporaryDirectory() as directory:
            pause = Path(directory, "rtcm.paused")
            pause.touch()
            self.source.pause_file = str(pause)
            request = self.request()
            self.source._service_writes(self.receiver, "session-a")
            self.assertIsInstance(request["error"], ForwardingBlocked)
            self.assertEqual(str(request["error"]), "corrections_paused")
            pause.unlink()
        self.source.current_gga["received_mono"] = time.monotonic() - 3
        request = self.request()
        self.source._service_writes(self.receiver, "session-a")
        self.assertIsInstance(request["error"], ForwardingBlocked)
        self.assertEqual(str(request["error"]), "serial_gga_not_fresh")
        self.source.current_gga.update(received_mono=time.monotonic(), raw=gga(quality=0))
        request = self.request()
        self.source._service_writes(self.receiver, "session-a")
        self.assertIsInstance(request["error"], ForwardingBlocked)
        self.assertEqual(str(request["error"]), "serial_gga_not_fresh")
        self.assertEqual(self.receiver.calls, [])

    def test_short_write_is_not_retried(self):
        receiver = Receiver(short=True)
        request = self.request()
        self.source._service_writes(receiver, "session-a")
        self.assertIsNone(request["error"])
        self.assertEqual(request["result"], len(frame()) - 1)
        self.source._service_writes(receiver, "session-a")
        self.assertEqual(len(receiver.calls), 1)

    def test_disconnect_clears_gga_and_wakes_pending_writer(self):
        request = self.request()
        self.source._clear_pending_writes()
        self.assertIsNone(self.source.get_gga())
        self.assertIsInstance(request["error"], ForwardingBlocked)
        self.assertEqual(str(request["error"]), "serial_disconnected")
        self.assertTrue(request["done"].is_set())
        self.assertTrue(self.source.write_requests.empty())
        self.assertEqual(self.receiver.calls, [])

    def test_repeated_epoch_cannot_refresh_write_gate(self):
        # The last advancing epoch is old; new serial receipts repeat that UTC.
        now, wall = time.monotonic(), time.time()
        tracker = FreshGgaTracker()
        for age in (4.0, 3.0):
            sample = {"raw": gga(wall=wall - age), "received_mono": now - age, "session": "session-a"}
            tracker.update(sample, now_mono=now - age, now_wall=wall - age)
        self.source.write_gga_tracker = tracker
        self.source.current_gga = {"raw": gga(wall=wall - 3.0), "received_mono": now, "session": "session-a"}
        request = self.request()
        self.source._service_writes(self.receiver, "session-a")
        self.assertIsInstance(request["error"], ForwardingBlocked)
        self.assertEqual(str(request["error"]), "serial_gga_not_advancing")
        self.assertEqual(self.receiver.calls, [])

    def test_current_gga_return_is_copy(self):
        sample = self.source.get_gga()
        sample["session"] = "modified-by-consumer"
        self.assertEqual(self.source.get_gga()["session"], "session-a")


if __name__ == "__main__":
    unittest.main()
