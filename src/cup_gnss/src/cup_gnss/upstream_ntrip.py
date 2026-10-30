"""NTRIP 1.4.1 library adapter for the existing single serial owner.

Network requests, authentication, GGA transmission, reconnect and RTCM decoding
run through LORD-MicroStrain/ntrip_client (MIT, ROS Noetic release 1.4.1).
The compatibility socket preserves the first RTCM bytes that upstream connect()
otherwise consumes, accepts fragmented status lines and checks HTTP headers
with Python's standard library. The strict delivery gate deliberately differs
from upstream's resynchronizing parser: no corrupt chunk or expired fragment
may release a valid prefix to this rover. No upstream ROS node/parameters are
created, so credentials stay in the existing mode-0600 file and process memory.
"""
import http.client
import io
import re
import socket
import time
import types
from urllib.parse import quote

from ntrip_client.ntrip_client import NTRIPClient as ReleasedNtripClient
from ntrip_client.rtcm_parser import RTCMParser

from .ntrip import NtripClient as SerialPolicyClient, NtripError, load_credentials

UPSTREAM_VERSION = '1.4.1'
UPSTREAM_COMMIT = 'ca4de0dad0fae12fa6d8e9856a993eec5007ce89'


def _quiet(*_args, **_kwargs):
    # Upstream logs include arbitrary HTTP responses and exception text. Never
    # forward those strings to ROS, stderr, bags or our diagnostic boundary.
    pass


class StrictUpstreamRtcmParser:
    """Bounded rover policy around the released upstream RTCM decoder.

    Upstream's caching/resynchronization is appropriate for generic streams but
    does not implement this project's all-or-nothing chunk/age contract. Hold
    incomplete bytes locally, verify with upstream's CRC implementation, then
    let its parser produce the complete packets. No second CRC implementation.
    """
    def __init__(self, owner=None, partial_timeout_s=2.0):
        self.owner = owner
        self.upstream = RTCMParser(_quiet, _quiet, _quiet, _quiet)
        self.pending = bytearray()
        self.partial_since = None
        self.partial_timeout_s = partial_timeout_s
        self.crc_failures = 0
        self.upstream_calls = 0

    def check_age(self, now):
        if self.partial_since is not None and now - self.partial_since > self.partial_timeout_s:
            self.pending.clear()
            self.partial_since = None
            raise NtripError('rtcm_partial_expired')

    def parse(self, data):
        now = time.monotonic()
        self.check_age(now)
        if len(data) > 4096:
            raise NtripError('rtcm_chunk_too_large')
        if self.owner:
            self.owner._count('rtcm_payload_bytes', len(data))
            self.owner._update(last_payload_mono=now)
        self.pending.extend(data)
        end = 0
        while end < len(self.pending):
            if self.pending[end] != 0xd3:
                raise NtripError('rtcm_invalid_frame')
            if len(self.pending) - end < 3:
                break
            length = ((self.pending[end + 1] & 3) << 8) | self.pending[end + 2]
            if self.pending[end + 1] & 0xfc or length < 2:
                raise NtripError('rtcm_invalid_frame')
            if len(self.pending) - end < length + 6:
                break
            packet = bytes(self.pending[end:end + length + 6])
            if self.upstream._checksum(packet[:-3]) != int.from_bytes(packet[-3:], 'big'):
                self.crc_failures += 1
                if self.owner: self.owner._count('crc_failures')
                raise NtripError('rtcm_crc_failure')
            end += length + 6
        packets = []
        if end:
            self.upstream_calls += 1
            packets = self.upstream.parse(bytes(self.pending[:end]))
            if b''.join(packets) != bytes(self.pending[:end]):
                raise NtripError('upstream_rtcm_mismatch')
            del self.pending[:end]
            self.partial_since = None
        if self.pending and self.partial_since is None:
            self.partial_since = now
        elif not self.pending:
            self.partial_since = None
        return packets


class _StatusSocket:
    """One socket shim; upstream still forms/sends/authenticates its request.

    The unbuffered, bounded first status line is returned alone to connect().
    Optional headers/body are read later, after the rover uploads its real GGA.
    HTTP header grammar is delegated to http.client rather than a local parser.
    """
    def __init__(self, owner, family, kind):
        self.owner = owner
        self.raw = socket.socket(family, kind)
        self.timeout = owner.connect_timeout_s
        self.status_sent = False
        self.headers_done = False
        self.header_buffer = bytearray()
        self.send_generation = 0
        with owner._lock: owner._socket = self

    def fileno(self): return self.raw.fileno()
    def settimeout(self, timeout):
        self.timeout = min(float(timeout), self.owner.connect_timeout_s)
        self.raw.settimeout(self.timeout)
    def shutdown(self, how): return self.raw.shutdown(how)
    def close(self): return self.raw.close()
    def connect(self, address):
        try:
            return self._connect(address)
        except NtripError as exc:
            self.owner._compat_error = str(exc)
            raise
        except OSError:
            self.owner._compat_error = 'network_connect_failed'
            raise

    def _connect(self, _address):
        deadline = time.monotonic() + self.owner.connect_timeout_s
        for family, _kind, _proto, _name, address in self.owner._resolve():
            if family != socket.AF_INET: continue
            self.owner._check_gate()
            remaining = deadline - time.monotonic()
            if remaining <= 0: raise NtripError('network_connect_timeout')
            self.raw.settimeout(remaining)
            self.raw.connect(address)
            self.raw.settimeout(min(.2, self.owner.connect_timeout_s))
            return
        raise NtripError('network_connect_failed')

    def send(self, data):
        self.owner._check_gate()
        self.raw.sendall(data)
        self.send_generation += 1
        return len(data)

    def _read(self, length, flags=0):
        data = self.raw.recv(length, flags)
        if data and not flags:
            self.owner._count('network_bytes', len(data))
            self.owner._update(last_network_mono=time.monotonic())
        return data

    def recv(self, size, flags=0):
        try:
            return self._recv(size, flags)
        except NtripError as exc:
            # recv_rtcm() catches socket exceptions broadly in the released
            # package. Preserve fixed policy reasons for the outer supervisor.
            self.owner._compat_error = str(exc)
            raise

    def _recv(self, size, flags=0):
        if flags: return self._read(size, flags)
        # Upstream drains while recv returns exactly its 1024-byte chunk. Keep
        # each fetch shorter so a busy caster cannot hide stop/GGA-age checks
        # behind an unbounded drain or accumulate a large stale backlog.
        size = min(size, 1023)
        if not self.status_sent:
            line = bytearray()
            deadline = time.monotonic() + self.owner.connect_timeout_s
            while not line.endswith(b'\r\n'):
                self.owner._check_gate()
                if len(line) >= 256 or time.monotonic() > deadline:
                    raise NtripError('invalid_status')
                try: byte = self._read(1)
                except socket.timeout: continue
                if not byte: raise NtripError('network_closed')
                line.extend(byte)
            if not re.fullmatch(rb'(ICY|HTTP/1\.[01]) [0-9]{3}(?: [\x20-\x7e]*)?\r\n', line):
                raise NtripError('invalid_status')
            self.owner._update(http_status=int(bytes(line).split(b' ', 2)[1]))
            self.status_sent = True
            return bytes(line)
        if self.headers_done:
            return self._read(size)
        while True:
            if self.header_buffer.startswith(b'\xd3'):
                self.headers_done = True
                result = bytes(self.header_buffer); self.header_buffer.clear()
                return result
            if self.header_buffer.startswith(b'\r\n'):
                self.headers_done = True
                result = bytes(self.header_buffer[2:]); self.header_buffer.clear()
                return result or self._read(size)
            ending = self.header_buffer.find(b'\r\n\r\n')
            if ending >= 0:
                block = bytes(self.header_buffer[:ending + 4])
                headers = http.client.parse_headers(io.BytesIO(block))
                for name in ('Transfer-Encoding', 'Content-Encoding', 'Content-Type'):
                    if len(headers.get_all(name, [])) > 1: raise NtripError('invalid_headers')
                if headers.get('Transfer-Encoding') is not None:
                    raise NtripError('unsupported_transfer_encoding')
                if headers.get('Content-Encoding', 'identity').lower() != 'identity':
                    raise NtripError('unsupported_content_encoding')
                content_type = headers.get('Content-Type', '').split(';')[0].strip().lower()
                if content_type and content_type not in {'gnss/data', 'application/octet-stream',
                        'application/rtcm', 'application/rtcm3', 'application/gnss'}:
                    raise NtripError('unexpected_content_type')
                self.headers_done = True
                result = bytes(self.header_buffer[ending + 4:]); self.header_buffer.clear()
                return result or self._read(size)
            if len(self.header_buffer) > 16384: raise NtripError('header_too_large')
            data = self._read(size)
            if not data: raise NtripError('network_closed')
            self.header_buffer.extend(data)


class _ReleasedClient(ReleasedNtripClient):
    def __init__(self, owner, username, password):
        super().__init__(owner.host, owner.port, quote(owner.mountpoint, safe='-._~'),
                         '', username, password, _quiet, _quiet, _quiet, _quiet)
        self.owner = owner
        # 1.4.1 checks attempt>=max BEFORE connect_success. max=1 therefore
        # throws even on its first successful reconnect. max=2 preserves that
        # successful socket; our connect() raises on failure, so no failed
        # attempt reaches upstream's retry/sleep branch (still one attempt).
        self.reconnect_attempt_max = 2
        self.reconnect_attempt_wait_seconds = 0
        self.rtcm_timeout_seconds = owner.network_idle_s
        # Long precision receiver GGA is already bounded/checksummed upstream of
        # this adapter. The generic 82-character default rejects its real data.
        self.nmea_parser.nmea_max_length = 1024
        # Reuse the exact released connect() code with a per-instance socket
        # factory. Its module/global socket object is never monkey-patched.
        # This narrow dependency injection is needed because 1.4.1 hardcodes
        # socket.socket; the factory fixes first-packet loss, not authentication.
        namespace = dict(ReleasedNtripClient.connect.__globals__)
        namespace['socket'] = types.SimpleNamespace(
            AF_INET=socket.AF_INET, SOCK_STREAM=socket.SOCK_STREAM,
            socket=lambda family, kind: _StatusSocket(owner, family, kind))
        self._released_connect = types.MethodType(types.FunctionType(
            ReleasedNtripClient.connect.__code__, namespace,
            ReleasedNtripClient.connect.__name__, ReleasedNtripClient.connect.__defaults__,
            ReleasedNtripClient.connect.__closure__), self)

    def connect(self):
        self.owner._compat_error = None
        self.owner._count('connection_attempts')
        self.owner._update(state='connecting', error='', http_status=None)
        self.rtcm_parser = StrictUpstreamRtcmParser(self.owner)
        self._first_rtcm_received = False
        self._read_zero_bytes_count = 0
        if not self._released_connect():
            self.disconnect()
            reason = self.owner._compat_error
            if not reason and self.owner._status.get('http_status') not in (None, 200):
                reason = 'http_rejected'
            raise NtripError(reason or 'ntrip_connect_rejected')
        self.owner._count('connections')
        self.owner._update(connected=True, state='streaming')
        self.send_nmea(self.owner._check_gate().decode('ascii'))
        return True

    def send_nmea(self, sentence):
        before = self._server_socket.send_generation
        super().send_nmea(sentence)
        if self._server_socket.send_generation == before:
            raise NtripError('gga_send_failed')
        self.owner._count('gga_sent')
        self.owner._update(last_gga_sent_mono=time.monotonic())

    def reconnect(self):
        if self.owner._compat_error:
            raise NtripError(self.owner._compat_error)
        self.owner._check_gate()
        self.owner._count('reconnects')
        # Invoke upstream reconnect, but never its uninterruptible retry sleep.
        # A failed single attempt returns to the outer stop-aware retry policy.
        return super().reconnect()


class _DecodedFrames:
    crc_failures = 0
    def __init__(self, packets): self.packets = packets
    def feed(self, _payload, _now): return self.packets


class UpstreamNtripClient(SerialPolicyClient):
    """Drop-in API; inherited methods are GGA/write/diagnostic policies only.

    Legacy _session/_connect/HTTP/CRC code is never called by this backend.
    No ROS publication or serial handle is added by the mature library.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.name = 'cup_gnss_ntrip_upstream'
        self._upstream = None
        self._compat_error = None
        self._update(backend='ntrip_client', upstream_version=UPSTREAM_VERSION,
                     upstream_commit=UPSTREAM_COMMIT)

    def _close_socket(self):
        super()._close_socket()
        client = getattr(self, '_upstream', None)
        if client is not None: client._connected = False

    def _upstream_session(self, username, password):
        client = _ReleasedClient(self, username, password)
        self._upstream = client
        client.connect()
        last_frame = time.monotonic()
        while not self.stop_event.is_set():
            current = self._check_gate()
            now = time.monotonic()
            last_gga = self._status['last_gga_sent_mono']
            if last_gga is None or now - last_gga >= self.gga_period_s:
                client.send_nmea(current.decode('ascii'))
            client.rtcm_parser.check_age(now)
            if now - last_frame > self.network_idle_s:
                # Use upstream's reconnect, not a private HTTP implementation.
                self._update(state='retrying', error='network_rtcm_timeout',
                             last_error='network_rtcm_timeout')
                client.reconnect()
                last_frame = time.monotonic()
            packets = client.recv_rtcm()
            if self._compat_error:
                raise NtripError(self._compat_error)
            if packets:
                last_frame = time.monotonic()
                # Preserve existing allowlist, per-frame GGA gate, exact UART
                # write accounting and fatal short-write contract. Already
                # decoded upstream packets are not parsed by the old framer.
                self._forward(b'', last_frame, _DecodedFrames(packets))
            self.stop_event.wait(.02)

    def run(self):
        try:
            while not self.stop_event.is_set():
                self._gga()
                if self._fatal:
                    self._update(state='fatal', fatal=True)
                    self.stop_event.wait(.1); continue
                if self._is_paused():
                    self._update(state='paused', error='')
                    self.stop_event.wait(.1); continue
                if not self._status['gga_available']:
                    self._update(state='waiting_gga', error=self._status['gga_error'])
                    self.stop_event.wait(.1); continue
                username = password = None
                try:
                    username, password = load_credentials(self.credentials_file)
                    self._upstream_session(username, password)
                except NtripError as exc:
                    reason = str(exc)
                    self._update(state='fatal' if self._fatal else 'retrying',
                                 error=reason, last_error=reason)
                except OSError:
                    self._update(state='retrying', error='network_io_failed', last_error='network_io_failed')
                except Exception:
                    self._update(state='retrying', error='ntrip_upstream_failed', last_error='ntrip_upstream_failed')
                finally:
                    username = password = None
                    self._close_socket()
                    if self._upstream is not None:
                        self._upstream._basic_credentials = None
                        self._upstream = None
                self._count('reconnects')
                self.stop_event.wait(self.retry_s)
        finally:
            self._close_socket()
            self._update(state='stopped')
