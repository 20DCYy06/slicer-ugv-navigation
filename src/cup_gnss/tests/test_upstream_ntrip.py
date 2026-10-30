"""TEST_ONLY: real released upstream library plus localhost caster, no UART."""
import contextlib
import io
import json
import os
from pathlib import Path
import pty
import socket
import sys
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
sys.path.insert(0, str(Path(__file__).parent))
from cup_gnss.upstream_ntrip import UpstreamNtripClient, StrictUpstreamRtcmParser
from cup_gnss.ntrip import NtripError
import test_ntrip as legacy
from test_ntrip import LoopbackCaster, frame, sample


class PlannedCaster(LoopbackCaster):
    """Each plan is one connection; failures are synthetic, never a real caster."""
    def run(self):
        self.upstream_requests = 0
        try:
            while not self.stop_event.is_set():
                try: connection, _ = self.server.accept()
                except socket.timeout: continue
                plan = self.packets[min(self.accepted, len(self.packets) - 1)]
                self.accepted += 1
                with connection:
                    connection.settimeout(.1)
                    request = b''
                    while b'\r\n\r\n' not in request and not self.stop_event.is_set():
                        data = connection.recv(4096)
                        if not data: break
                        request += data
                    if b'User-Agent: NTRIP ntrip_client_ros\r\n' in request:
                        self.upstream_requests += 1
                    for part in plan.get('response', [b'ICY 200 OK\r\n']):
                        connection.sendall(part)
                        self.stop_event.wait(.005)
                    pending = b''
                    deadline = time.monotonic() + 1
                    while b'\r\n' not in pending and time.monotonic() < deadline and not self.stop_event.is_set():
                        try: data = connection.recv(4096)
                        except socket.timeout: continue
                        if not data: break
                        pending += data
                    if pending: self.ggas.append(pending)
                    for chunk in plan.get('chunks', []):
                        connection.sendall(chunk)
                        self.stop_event.wait(.01)
                    if plan.get('close'): continue
                    while not self.stop_event.is_set():
                        try:
                            if not connection.recv(4096): break
                        except socket.timeout: continue
        except OSError:
            pass
        except Exception as exc:
            self.error = exc
        finally:
            self.server.close()


class UpstreamPolicyCase(legacy.ClientCase):
    def client(self, packets, writer, pause_file=None, frame_callback=None):
        caster = LoopbackCaster(packets)
        self.casters.append(caster)
        client = UpstreamNtripClient(
            str(self.credentials), lambda: sample(time.time() % 86400, time.monotonic()),
            writer, host='127.0.0.1', port=caster.port, retry_s=.05,
            connect_timeout_s=1, pause_file=pause_file, frame_callback=frame_callback)
        self.clients.append(client)
        client.start()
        return client, caster

    def planned_client(self, plans, writer, **kwargs):
        caster = PlannedCaster(plans)
        self.casters.append(caster)
        client = UpstreamNtripClient(
            str(self.credentials), lambda: sample(time.time() % 86400, time.monotonic()),
            writer, host='127.0.0.1', port=caster.port, retry_s=.05,
            connect_timeout_s=.5, **kwargs)
        self.clients.append(client); client.start()
        return client, caster

    def test_fragmented_http_and_same_packet_rtcm_are_not_dropped(self):
        written = []
        packet = frame(1005)
        plans = [{'response': [b'H', b'TTP/1.0 200 O',
                  b'K\r\nContent-Type: gnss/data\r\n\r\n' + packet]}]
        client, caster = self.planned_client(plans, lambda data: written.append(data) or len(data))
        self.wait_for(lambda: bool(written))
        self.assertEqual(written, [packet])
        self.assertEqual(caster.upstream_requests, 1)
        self.assertEqual(client.snapshot()['backend'], 'ntrip_client')

    def test_upstream_reconnect_reuploads_receiver_gga_and_recovers(self):
        written = []
        one, two = frame(1005), frame(1074)
        client, caster = self.planned_client([
            {'chunks': [one], 'close': True}, {'chunks': [two]}],
            lambda data: written.append(data) or len(data))
        self.wait_for(lambda: len(written) >= 2)
        self.assertEqual(written[:2], [one, two])
        self.assertGreaterEqual(client.snapshot()['reconnects'], 1)
        # The first successful reconnect must remain in service. Upstream
        # 1.4.1 tests attempt>=max before success, so max=1 wrongly throws even
        # after its successful second socket and forces a third connection.
        time.sleep(.1)
        self.assertEqual(client.snapshot()['connections'], 2)
        self.assertEqual(caster.accepted, 2)
        self.assertEqual(len(caster.ggas), 2)
        self.assertTrue(all(raw.startswith(b'$GNGGA,') for raw in caster.ggas))
        self.assertEqual(caster.upstream_requests, caster.accepted)

    def test_unauthorized_reason_is_fixed_and_does_not_echo_server_text(self):
        written = []
        client, _ = self.planned_client([{'response': [
            b'HTTP/1.0 401 unit-password\r\n']}],
            lambda data: written.append(data) or len(data))
        self.wait_for(lambda: client.snapshot()['last_error'] == 'http_rejected')
        state = client.snapshot()
        self.assertEqual(state['http_status'], 401)
        self.assertEqual(written, [])
        self.assertNotIn('unit-password', json.dumps(state))

    def test_unsupported_http_encoding_fails_before_any_uart_write(self):
        written = []
        client, _ = self.planned_client([{'response': [
            b'HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n' + frame()]}],
            lambda data: written.append(data) or len(data))
        self.wait_for(lambda: bool(client.snapshot()['last_error']))
        self.assertEqual(written, [])
        self.assertFalse(client.snapshot()['fatal'])

    def test_old_http_and_rtcm_protocol_stack_is_not_called(self):
        from unittest import mock
        from cup_gnss.ntrip import NtripClient, NtripResponseParser, RtcmFramer
        written = []
        with mock.patch.object(NtripClient, '_session', side_effect=AssertionError('legacy stack')), \
             mock.patch.object(NtripClient, '_connect', side_effect=AssertionError('legacy stack')), \
             mock.patch.object(NtripResponseParser, 'feed', side_effect=AssertionError('legacy stack')), \
             mock.patch.object(RtcmFramer, 'feed', side_effect=AssertionError('legacy stack')):
            client, _ = self.client([frame()], lambda data: written.append(data) or len(data))
            self.wait_for(lambda: bool(written))
            client.stop(); client.join(2)
        self.assertEqual(written, [frame()])

    def test_failed_upstream_reconnect_is_one_attempt_without_library_sleep(self):
        from unittest import mock
        from types import SimpleNamespace
        import ntrip_client.ntrip_base as released_base
        from cup_gnss.upstream_ntrip import _ReleasedClient
        owner = UpstreamNtripClient(str(self.credentials), lambda: None, lambda frame: len(frame),
                                    host='127.0.0.1', port=1)
        # Only the network establishment result is stubbed here. The released
        # reconnect and our failure boundary run unchanged, so max=2 cannot
        # silently introduce a second attempt or uninterruptible library sleep.
        owner._check_gate = lambda: b'$TEST_ONLY\r\n'
        client = _ReleasedClient(owner, 'unit-user', 'unit-password')
        client._connected = True
        client._released_connect = lambda: False
        no_sleep = mock.Mock(side_effect=AssertionError('unbounded upstream sleep'))
        with mock.patch.object(released_base, 'time', SimpleNamespace(sleep=no_sleep)):
            with self.assertRaisesRegex(NtripError, '^ntrip_connect_rejected$'):
                client.reconnect()
        self.assertEqual(owner.snapshot()['connection_attempts'], 1)
        no_sleep.assert_not_called()

    def test_upstream_parser_used_for_partial_crc_and_no_good_prefix_escape(self):
        parser = StrictUpstreamRtcmParser()
        one = frame(1074)
        self.assertEqual(parser.parse(one[:1]), [])
        self.assertEqual(parser.parse(one[1:5]), [])
        self.assertEqual(parser.parse(one[5:]), [one])
        bad = bytearray(frame(1084)); bad[-1] ^= 1
        with self.assertRaisesRegex(NtripError, '^rtcm_crc_failure$'):
            parser.parse(one + bad)
        self.assertEqual(parser.upstream_calls, 1)
        self.assertEqual(parser.crc_failures, 1)

    def test_large_frame_fragmentation_and_partial_timeout_are_explicit(self):
        parser = StrictUpstreamRtcmParser()
        packet = frame(1074, bytes(range(256)) * 3)
        self.assertEqual(parser.parse(packet[:200]), [])
        self.assertEqual(parser.parse(packet[200:]), [packet])
        parser.parse(packet[:1])
        parser.partial_since = time.monotonic() - 2.01
        with self.assertRaisesRegex(NtripError, '^rtcm_partial_expired$'):
            parser.parse(packet[1:])

    def test_gga_expiration_closes_stream_without_replaying_last_upload(self):
        written = []
        state = {'active': True}
        caster = PlannedCaster([{'chunks': [frame()]}]); self.casters.append(caster)
        client = UpstreamNtripClient(str(self.credentials),
            lambda: sample(time.time() % 86400, time.monotonic()) if state['active'] else None,
            lambda data: written.append(data) or len(data), host='127.0.0.1',
            port=caster.port, retry_s=.05, connect_timeout_s=.5, gga_period_s=.05)
        self.clients.append(client); client.start()
        self.wait_for(lambda: bool(written))
        state['active'] = False
        self.wait_for(lambda: client.snapshot()['state'] == 'waiting_gga')
        accepted, count = caster.accepted, client.snapshot()['gga_sent']
        time.sleep(.15)
        self.assertEqual(caster.accepted, accepted)
        self.assertEqual(client.snapshot()['gga_sent'], count)
        self.assertFalse(client.snapshot()['connected'])

    def test_real_pty_write_bytes_are_identical(self):
        master, slave = pty.openpty()
        import tty
        tty.setraw(slave)
        os.set_blocking(master, False)
        one, two = frame(1005), frame(1074)
        try:
            client, _ = self.client([one[:2], one[2:] + two], lambda data: os.write(slave, data))
            received = bytearray()
            def complete():
                try: received.extend(os.read(master, 4096))
                except BlockingIOError: pass
                return len(received) >= len(one + two)
            self.wait_for(complete)
            self.assertEqual(bytes(received), one + two)
            # The kernel can expose written PTY bytes before the writer returns
            # and the NTRIP thread updates its acknowledged-write counters.
            self.wait_for(lambda: client.snapshot()['written_bytes'] == len(one + two))
            self.assertEqual(client.snapshot()['written_bytes'], len(one + two))
        finally:
            for client in self.clients: client.stop(); client.join(2)
            os.close(master); os.close(slave)

    def test_exception_and_upstream_response_text_never_reach_diagnostics(self):
        capture = io.StringIO()
        with contextlib.redirect_stderr(capture), contextlib.redirect_stdout(capture):
            client, _ = self.client([frame()], lambda data: (_ for _ in ()).throw(RuntimeError('unit-password')))
            self.wait_for(lambda: client.snapshot()['fatal'])
        self.assertNotIn('unit-password', capture.getvalue() + json.dumps(client.snapshot()))
        self.assertEqual(client.snapshot()['last_error'], 'serial_write_failed')


if __name__ == '__main__': unittest.main()
