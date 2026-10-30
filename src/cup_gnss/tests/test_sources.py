"""Local synthetic transport unit fixtures, separate from real-device acceptance."""
import json
from pathlib import Path
import queue
import socket
import sys
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from cup_gnss.sources import LiveSource, SourceProtocolError, TcpSource


class TcpSourceCase(unittest.TestCase):
    def exchange(self, payload):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        server.settimeout(3)
        port = server.getsockname()[1]
        finished = threading.Event()

        def send():
            try:
                connection, _ = server.accept()
                with connection:
                    for offset in range(0, len(payload), 17):
                        connection.sendall(payload[offset:offset + 17])
                    finished.wait(1.0)
            finally:
                server.close()

        thread = threading.Thread(target=send, daemon=True)
        thread.start()
        events = queue.Queue(maxsize=32)
        source = TcpSource(events, host="127.0.0.1", port=port, retry_s=10)
        source.start()
        collected = []
        try:
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                event = events.get(timeout=3)
                collected.append(event)
                if event["kind"] == "nmea" or (event["kind"] == "transport" and not event["connected"]):
                    break
        finally:
            source.stop()
            finished.set()
            source.join(3)
            thread.join(3)
        self.assertFalse(source.is_alive())
        self.assertFalse(thread.is_alive())
        return collected

    def test_split_json_records_and_wire_receipt_metadata_overridden(self):
        packet = {"v": 1, "kind": "nmea", "session": "unit-only", "seq": 1,
                  "captured_at": time.time(), "nmea": "$GNGGA,,,,,,0,,,,,,,,*78",
                  "_received_wall": -100, "_received_mono": -100}
        events = self.exchange((json.dumps(packet) + "\n").encode())
        self.assertTrue(events[0]["connected"])
        received = [item for item in events if item["kind"] == "nmea"][0]
        self.assertEqual(received["nmea"], packet["nmea"])
        self.assertGreater(received["_received_wall"], 1000)
        self.assertGreater(received["_received_mono"], 0)

    def test_bad_json_disconnects_with_protocol_diagnostic(self):
        events = self.exchange(b"[]\n")
        self.assertFalse(events[-1]["connected"])
        self.assertEqual(events[-1]["error"], "invalid_tcp_record_kind")

    def test_oversized_tcp_record_is_bounded(self):
        events = self.exchange(b"x" * 4097)
        self.assertFalse(events[-1]["connected"])
        self.assertEqual(events[-1]["error"], "tcp_record_too_large")


class QueueCase(unittest.TestCase):
    def test_overflow_discards_backlog_and_marks_disconnected(self):
        events = queue.Queue(maxsize=1)
        source = LiveSource(events)
        source.emit({"kind": "nmea", "nmea": "offline-old"})
        with self.assertRaises(SourceProtocolError):
            source.emit({"kind": "nmea", "nmea": "offline-new"})
        event = events.get_nowait()
        self.assertEqual(event["kind"], "transport")
        self.assertFalse(event["connected"])
        self.assertEqual(event["error"], "receive_queue_overflow")
        self.assertTrue(events.empty())


if __name__ == "__main__":
    unittest.main()
