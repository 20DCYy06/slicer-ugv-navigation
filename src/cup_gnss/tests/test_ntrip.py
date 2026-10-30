"""Synthetic protocol/loopback tests; these are not receiver acceptance."""
import json
import os
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from cup_gnss.ntrip import (ALLOWED_RTCM_IDS, ForwardingBlocked, FreshGgaTracker,
                            NtripClient, NtripError, NtripResponseParser,
                            RtcmFramer, crc24q, load_credentials)


def sentence(body):
    checksum = 0
    for byte in body.encode("ascii"):
        checksum ^= byte
    return "$" + body + "*%02X" % checksum


def gga(epoch, quality=1):
    epoch %= 86400
    hours, remaining = divmod(epoch, 3600)
    minutes, seconds = divmod(remaining, 60)
    stamp = "%02d%02d%06.3f" % (hours, minutes, seconds)
    return sentence("GNGGA," + stamp + ",3112.123456,N,12130.654321,E,%d,20,0.7,15.0,M,0.0,M,1.0,0001" % quality)


def frame(message=1074, suffix=b"\x00\x01\x02\x03"):
    payload = bytes((message >> 4, (message & 15) << 4)) + suffix
    packet = b"\xd3" + bytes((len(payload) >> 8, len(payload) & 255)) + payload
    return packet + crc24q(packet).to_bytes(3, "big")


def sample(epoch, receipt, session="unit-only", quality=1):
    return {"raw": gga(epoch, quality), "received_mono": receipt, "session": session}


class ProtocolCase(unittest.TestCase):
    def test_crc24q_standard_check_vector(self):
        self.assertEqual(crc24q(b"123456789"), 0xCDE703)

    def test_framer_split_frames_and_mixed_message_ids(self):
        framer = RtcmFramer()
        one, two = frame(1005), frame(1124)
        self.assertEqual(framer.feed(one[:2], now=10), [])
        self.assertEqual(framer.feed(one[2:8], now=10.2), [])
        self.assertEqual(framer.feed(one[8:] + two, now=10.4), [one, two])
        self.assertEqual(framer.pending, b"")

    def test_corrupt_chunk_never_releases_its_good_prefix(self):
        framer = RtcmFramer()
        bad = bytearray(frame())
        bad[-1] ^= 1
        with self.assertRaisesRegex(NtripError, "^rtcm_crc_failure$"):
            framer.feed(frame(1005) + bad)
        self.assertEqual(framer.crc_failures, 1)

    def test_partial_expiration_and_malformed_headers_fail_closed(self):
        framer = RtcmFramer()
        framer.feed(frame()[:4], now=10)
        with self.assertRaisesRegex(NtripError, "^rtcm_partial_expired$"):
            framer.feed(frame()[4:], now=12.01)
        for bad in (b"garbage", b"\xd3\xfc\x02", b"\xd3\x00\x01z"):
            with self.subTest(bad=bad), self.assertRaisesRegex(NtripError, "^rtcm_invalid_frame$"):
                RtcmFramer().feed(bad)

    def test_headerless_icy_and_split_http_headers(self):
        payload = frame()
        for prefix in (b"ICY 200 OK\r\n", b"ICY 200 OK\r\n\r\n",
                       b"HTTP/1.0 200 OK\r\nContent-Type: gnss/data\r\n\r\n"):
            parser, result = NtripResponseParser(), bytearray()
            for byte in prefix + payload:
                result.extend(parser.feed(bytes((byte,))))
            self.assertTrue(parser.ready)
            self.assertEqual(bytes(result), payload)

    def test_no_http_error_sourcetable_or_encoding_reaches_rtcm(self):
        cases = ((b"HTTP/1.0 401 Unauthorized\r\n", "http_rejected"),
                 (b"SOURCETABLE 200 OK\r\n", "sourcetable_response"),
                 (b"HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n", "unsupported_transfer_encoding"),
                 (b"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n", "unexpected_content_type"),
                 (b"ICY 200 OK\r\n\r\n<html>failure</html>", "unexpected_body"))
        for data, reason in cases:
            with self.subTest(reason=reason), self.assertRaisesRegex(NtripError, "^" + reason + "$"):
                NtripResponseParser().feed(data)


class GgaCase(unittest.TestCase):
    def test_two_advancing_fixes_and_midnight_wrap(self):
        tracker = FreshGgaTracker()
        self.assertIsNone(tracker.update(sample(86399, 10), 10, 86399))
        self.assertEqual(tracker.update(sample(0, 11), 11, 86400), (gga(0) + "\r\n").encode())
        self.assertEqual(tracker.error, "")

    def test_duplicate_epoch_cannot_refresh_receipt_or_keep_stale_fix(self):
        tracker = FreshGgaTracker()
        tracker.update(sample(100, 10), 10, 100)
        tracker.update(sample(101, 11), 11, 101)
        self.assertIsNotNone(tracker.update(sample(101, 12), 12, 102))
        self.assertIsNone(tracker.update(sample(101, 14), 14, 104))
        self.assertEqual(tracker.error, "gga_not_advancing")

    def test_bad_fix_clock_session_old_sample_and_reversed_epoch_invalidate(self):
        cases = ((sample(102, 12, quality=0), 12, 102, "gga_no_valid_fix"),
                 (sample(102, 12), 12, 200, "gga_clock_mismatch"),
                 (sample(102, 12, session="different"), 12, 102, "waiting_advancing_gga"),
                 (sample(102, 1), 12, 102, "gga_expired"),
                 (sample(99, 12), 12, 102, "waiting_advancing_gga"))
        for current, mono, wall, error in cases:
            tracker = FreshGgaTracker()
            tracker.update(sample(100, 10), 10, 100)
            tracker.update(sample(101, 11), 11, 101)
            with self.subTest(error=error):
                self.assertIsNone(tracker.update(current, mono, wall))
                self.assertEqual(tracker.error, error)


class CredentialsCase(unittest.TestCase):
    def test_owned_regular_0600_only_and_no_content_in_errors(self):
        with tempfile.TemporaryDirectory(prefix="cup_ntrip_test_") as temporary:
            path = Path(temporary) / "credentials.json"
            path.write_text(json.dumps({"username": "synthetic-user", "password": "synthetic-secret"}))
            path.chmod(0o600)
            self.assertEqual(load_credentials(str(path)), ("synthetic-user", "synthetic-secret"))
            path.chmod(0o644)
            with self.assertRaisesRegex(NtripError, "^credentials_permissions$"):
                load_credentials(str(path))
            path.chmod(0o600)
            link = Path(temporary) / "link"
            link.symlink_to(path)
            with self.assertRaisesRegex(NtripError, "^credentials_unreadable$"):
                load_credentials(str(link))
            path.write_text("synthetic-secret malformed JSON")
            with self.assertRaisesRegex(NtripError, "^invalid_credentials$"):
                load_credentials(str(path))


class LoopbackCaster:
    """One local synthetic caster; no internet or real account is accessed."""
    def __init__(self, packets):
        self.server = socket.socket()
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("127.0.0.1", 0))
        self.server.listen(4)
        self.server.settimeout(0.1)
        self.port = self.server.getsockname()[1]
        self.packets = packets
        self.stop_event = threading.Event()
        self.accepted = 0
        self.ggas = []
        self.error = None
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        try:
            while not self.stop_event.is_set():
                try:
                    connection, _ = self.server.accept()
                except socket.timeout:
                    continue
                self.accepted += 1
                with connection:
                    connection.settimeout(0.2)
                    request = b""
                    while b"\r\n\r\n" not in request and not self.stop_event.is_set():
                        request += connection.recv(4096)
                    if not request.startswith(b"GET /AUTO HTTP/1.0\r\n"):
                        raise AssertionError("wrong synthetic mountpoint request")
                    connection.sendall(b"ICY 200 OK\r\n")
                    pending = b""
                    # Deliberately send no headers/body before receiving GGA.
                    deadline = time.monotonic() + 2.0
                    while b"\r\n" not in pending and time.monotonic() < deadline:
                        try:
                            data = connection.recv(4096)
                        except socket.timeout:
                            continue
                        if not data:
                            break
                        pending += data
                    if pending:
                        self.ggas.append(pending)
                    if not pending.startswith(b"$GNGGA,"):
                        continue
                    for chunk in self.packets:
                        if self.stop_event.wait(0.01):
                            return
                        connection.sendall(chunk)
                    while not self.stop_event.is_set():
                        try:
                            if not connection.recv(4096):
                                break
                        except socket.timeout:
                            continue
        except OSError:
            if not self.stop_event.is_set():
                # A client intentionally closes immediately at unsafe frames.
                pass
        except Exception as exc:
            self.error = exc
        finally:
            self.server.close()

    def stop(self):
        self.stop_event.set()
        self.thread.join(2)


class ClientCase(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="cup_ntrip_client_test_")
        self.directory = Path(self.temporary.name)
        self.credentials = self.directory / "credentials.json"
        self.credentials.write_text(json.dumps({"username": "unit-user", "password": "unit-password"}))
        self.credentials.chmod(0o600)
        self.clients = []
        self.casters = []

    def tearDown(self):
        for client in self.clients:
            client.stop()
            client.join(2)
            self.assertFalse(client.is_alive(), "NTRIP client stop must be bounded")
        for caster in self.casters:
            caster.stop()
            self.assertFalse(caster.thread.is_alive())
            if caster.error:
                raise caster.error
        self.temporary.cleanup()

    def client(self, packets, writer, pause_file=None, frame_callback=None):
        caster = LoopbackCaster(packets)
        self.casters.append(caster)
        def provider():
            return sample(time.time() % 86400, time.monotonic())
        client = NtripClient(str(self.credentials), provider, writer, host="127.0.0.1",
                             port=caster.port, pause_file=pause_file, retry_s=0.05,
                             connect_timeout_s=1.0, frame_callback=frame_callback)
        self.clients.append(client)
        client.start()
        return client, caster

    def wait_for(self, predicate, seconds=3.0):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        self.fail("synthetic transport condition not reached")

    def test_live_vrs_gga_then_complete_allowlisted_frames_only(self):
        written, evidence = [], []
        one, unsupported, two = frame(1005), frame(1230), frame(1124)
        def write(data):
            written.append(data)
            return len(data)
        client, caster = self.client([one[:1], one[1:] + unsupported + two], write,
                                      frame_callback=evidence.append)
        self.wait_for(lambda: len(written) == 2)
        client.stop()
        client.join(2)
        state = client.snapshot()
        self.assertEqual(written, [one, two])
        self.assertGreaterEqual(state["gga_sent"], 1)
        self.assertEqual(state["received_frames"], 3)
        self.assertEqual(state["written_frames"], 2)
        self.assertEqual(state["skipped_types"], {"1230": 1})
        self.assertEqual(state["written_types"], {"1005": 1, "1124": 1})
        self.assertIsNotNone(state["network_age_s"])
        self.assertIsNotNone(state["write_age_s"])
        self.assertTrue(caster.ggas)
        self.assertEqual(len([entry for entry in evidence if entry["direction"] == "tx"]), 2)
        self.assertNotIn("unit-password", json.dumps(state))
        self.assertEqual(set(state["allowed_types"]), ALLOWED_RTCM_IDS)

    def test_short_write_latches_fatal_and_never_finishes_or_sends_next_frame(self):
        written = []
        def write(data):
            written.append(data)
            return len(data) - 1
        client, caster = self.client([frame(1074) + frame(1084)], write)
        self.wait_for(lambda: client.snapshot()["fatal"])
        time.sleep(0.15)
        state = client.snapshot()
        self.assertEqual(len(written), 1)
        self.assertEqual(caster.accepted, 1)
        self.assertEqual(state["short_writes"], 1)
        self.assertEqual(state["written_frames"], 0)
        self.assertEqual(state["last_write_error"], "serial_partial_write")
        self.assertIsNone(state["last_write_mono"])
        self.assertIsNone(state["write_age_s"])
        self.assertIsNotNone(state["last_write_attempt_mono"])
        client.pause()
        client.resume()
        time.sleep(0.15)
        self.assertTrue(client.snapshot()["fatal"])
        self.assertEqual(len(written), 1)

    def test_bad_crc_chunk_cannot_write_good_prefix(self):
        written = []
        bad = bytearray(frame(1084))
        bad[-1] ^= 1
        client, _ = self.client([frame(1074) + bad], lambda data: written.append(data) or len(data))
        self.wait_for(lambda: client.snapshot()["crc_failures"] >= 1)
        client.stop()
        self.assertEqual(written, [])
        self.assertEqual(client.snapshot()["last_error"], "rtcm_crc_failure")

    def test_pause_file_blocks_connection_and_midframe_pause_blocks_write(self):
        pause = self.directory / "paused"
        pause.touch()
        written = []
        def callback(event):
            if event["direction"] == "rx":
                pause.touch()
        client, caster = self.client([frame()], lambda data: written.append(data) or len(data),
                                      pause_file=str(pause), frame_callback=callback)
        self.wait_for(lambda: client.snapshot()["state"] == "paused")
        self.assertEqual(caster.accepted, 0)
        pause.unlink()
        self.wait_for(lambda: client.snapshot()["received_frames"] == 1)
        self.wait_for(lambda: client.snapshot()["state"] == "paused")
        self.assertEqual(written, [])
        self.assertFalse(client.snapshot()["fatal"])

    def test_prewrite_refusal_is_recoverable_without_claiming_bytes_written(self):
        calls = []
        def write(data):
            calls.append(data)
            raise ForwardingBlocked()
        client, _ = self.client([frame()], write)
        self.wait_for(lambda: bool(calls))
        client.stop()
        state = client.snapshot()
        self.assertFalse(state["fatal"])
        self.assertEqual(state["written_bytes"], 0)
        self.assertEqual(state["last_error"], "forwarding_gate_closed")
        self.assertIsNone(state["last_write_mono"])
        self.assertIsNone(state["write_age_s"])


if __name__ == "__main__":
    unittest.main()
