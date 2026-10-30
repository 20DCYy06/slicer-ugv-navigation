"""Serial/GGA policy primitives and retired legacy NTRIP transport.

The live default is upstream_ntrip.UpstreamNtripClient. The old request/HTTP/
CRC implementation remains marked legacy for historical regression only;
production networking and decoding do not call it. Retained project policy
checks receiver/session freshness, allowlisted bytes and exact UART writes.

No receiver configuration, replay, second serial handle, or credential logging.
The response handling, CRC24Q, GGA checks, and message allowlist preserve the
Windows diagnostics that succeeded with this dongle. Non-TLS is an explicitly
accepted field-test setting. It is not advertised as encrypted transport.
"""
import base64
import collections
import json
import math
import os
import re
import socket
import stat
import threading
import time
from urllib.parse import quote

from .core import parse_nmea


ALLOWED_RTCM_IDS = frozenset({1005, 1033, 1074, 1084, 1094, 1114, 1124})


class NtripError(Exception):
    """Only fixed, non-secret reason codes cross this boundary."""


class ForwardingBlocked(Exception):
    """The serial owner refused a write BEFORE calling UART.write()."""


def crc24q(data):
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def load_credentials(path):
    """Open once, reject symlinks/races, and require an owned mode-0600 file."""
    try:
        flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_NONBLOCK", 0)
        descriptor = os.open(path, flags)
        with os.fdopen(descriptor, "rb") as handle:
            info = os.fstat(handle.fileno())
            if (not stat.S_ISREG(info.st_mode) or stat.S_IMODE(info.st_mode) != 0o600
                    or info.st_uid != os.geteuid()):
                raise NtripError("credentials_permissions")
            raw = handle.read(8193)
        if len(raw) > 8192:
            raise NtripError("invalid_credentials")
        value = json.loads(raw.decode("utf-8"))
        username, password = value["username"], value["password"]
        if (not isinstance(username, str) or not isinstance(password, str)
                or not username or not password or ":" in username
                or any(ord(char) < 32 or ord(char) > 126 for char in username + password)):
            raise NtripError("invalid_credentials")
        return username, password
    except NtripError:
        raise
    except FileNotFoundError:
        raise NtripError("credentials_missing") from None
    except OSError:
        raise NtripError("credentials_unreadable") from None
    except (ValueError, TypeError, KeyError, UnicodeError):
        raise NtripError("invalid_credentials") from None


def validate_endpoint(host, port, mountpoint):
    if not re.fullmatch(r"[A-Za-z0-9.-]+", host):
        raise ValueError("invalid NTRIP host")
    if isinstance(port, bool) or not isinstance(port, int) or not 0 < port < 65536:
        raise ValueError("invalid NTRIP port")
    if not isinstance(mountpoint, str) or not mountpoint or len(mountpoint) > 256:
        raise ValueError("invalid NTRIP mountpoint")


def build_request(host, port, mountpoint, username, password):
    """Legacy request builder; not used by the mature live backend."""
    validate_endpoint(host, port, mountpoint)
    try:
        token = base64.b64encode((username + ":" + password).encode("ascii"))
    except (UnicodeError, TypeError):
        raise NtripError("invalid_credentials") from None
    return (b"GET /" + quote(mountpoint, safe="-._~").encode("ascii") + b" HTTP/1.0\r\n"
            b"Host: " + host.encode("ascii") + b":" + str(port).encode("ascii") + b"\r\n"
            b"User-Agent: NTRIP cup_gnss/1.0\r\n"
            b"Accept: */*\r\nConnection: close\r\nAuthorization: Basic "
            + token + b"\r\n\r\n")


class FreshGgaTracker:
    """Two advancing fixes, UTC agreement, receipt age, and session boundaries.

    Provider samples are {raw, received_mono, session}. Polling the same sample
    cannot refresh it. A current no-fix GGA immediately invalidates a prior fix.
    A same-epoch fix may retain its original age but cannot extend that age.
    """
    def __init__(self, max_age_s=2.5, utc_tolerance_s=15.0):
        self.max_age_s = max_age_s
        self.utc_tolerance_s = utc_tolerance_s
        self.session = None
        self.previous_epoch = None
        self.latest = None
        self.latest_mono = None
        self.error = "waiting_gga"

    def _invalidate(self, error, reset_epoch=True):
        self.latest = self.latest_mono = None
        if reset_epoch:
            self.previous_epoch = None
        self.error = error
        return None

    def update(self, sample, now_mono=None, now_wall=None):
        now = time.monotonic() if now_mono is None else now_mono
        wall = time.time() if now_wall is None else now_wall
        if not isinstance(sample, dict):
            return self._invalidate("waiting_gga")
        raw, receipt, session = (sample.get("raw"), sample.get("received_mono"),
                                 sample.get("session"))
        if (not isinstance(session, str) or not session or len(session) > 128
                or isinstance(receipt, bool) or not isinstance(receipt, (float, int))
                or not math.isfinite(receipt) or receipt > now + 0.01):
            return self._invalidate("invalid_gga_sample")
        if now - receipt > self.max_age_s:
            return self._invalidate("gga_expired")
        if session != self.session:
            self._invalidate("waiting_advancing_gga")
            self.session = session
        record = parse_nmea(raw)
        if record is None or record["kind"] != "GGA":
            return self._invalidate("invalid_gga")
        epoch = record.get("epoch")
        if (record.get("quality") not in {1, 2, 4, 5}
                or record.get("latitude") is None or record.get("longitude") is None
                or epoch is None):
            return self._invalidate("gga_no_valid_fix")
        clock_delta = abs((epoch - wall % 86400 + 43200) % 86400 - 43200)
        if clock_delta > self.utc_tolerance_s:
            return self._invalidate("gga_clock_mismatch")
        delta = None if self.previous_epoch is None else (epoch - self.previous_epoch) % 86400
        if delta is not None and 0 < delta <= 5.0:
            self.latest = raw.encode("ascii") + b"\r\n"
            self.latest_mono = receipt
            self.error = ""
        elif delta != 0:
            self._invalidate("waiting_advancing_gga", reset_epoch=False)
        self.previous_epoch = epoch
        if self.latest_mono is None:
            return None
        if now - self.latest_mono > self.max_age_s:
            return self._invalidate("gga_not_advancing")
        return self.latest


class NtripResponseParser:
    """Bounded v1/HTTP response parser, including ICY with no header block."""
    def __init__(self):
        self.pending = b""
        self.stage = "status"
        self.status = None
        self.icy = False
        self.ready = False
        self.body_checked = False

    def feed(self, data):
        if self.stage == "body" and self.body_checked:
            return data
        self.pending += data
        if self.stage == "status":
            ending = self.pending.find(b"\r\n")
            if ending < 0:
                if len(self.pending) > 256:
                    raise NtripError("invalid_status")
                return b""
            if ending > 256:
                raise NtripError("invalid_status")
            line, self.pending = self.pending[:ending], self.pending[ending + 2:]
            if line.startswith(b"SOURCETABLE"):
                raise NtripError("sourcetable_response")
            match = re.fullmatch(rb"(ICY|HTTP/1\.[01]) ([0-9]{3})(?: [\x20-\x7e]*)?", line)
            if not match:
                raise NtripError("invalid_status")
            self.status = int(match[2])
            self.icy = match[1] == b"ICY"
            if self.status != 200:
                raise NtripError("http_rejected")
            self.stage = "headers"
        if self.stage == "headers":
            if self.icy and self.pending.startswith((b"SOURCETABLE", b"STR;", b"CAS;", b"NET;")):
                raise NtripError("sourcetable_response")
            if self.icy and self.pending.startswith(b"\xd3"):
                self.stage, self.ready = "body", True
            else:
                if self.pending.startswith(b"\r\n"):
                    header_block, self.pending = b"", self.pending[2:]
                else:
                    ending = self.pending.find(b"\r\n\r\n")
                    if ending < 0:
                        if len(self.pending) > 16384:
                            raise NtripError("header_too_large")
                        return b""
                    if ending > 16384:
                        raise NtripError("header_too_large")
                    header_block, self.pending = self.pending[:ending], self.pending[ending + 4:]
                headers = {}
                for line in header_block.split(b"\r\n") if header_block else []:
                    if b":" not in line:
                        raise NtripError("invalid_headers")
                    key, value = line.split(b":", 1)
                    if not re.fullmatch(rb"[A-Za-z0-9-]+", key):
                        raise NtripError("invalid_headers")
                    key, value = key.lower(), value.strip().lower()
                    if key in headers:
                        raise NtripError("invalid_headers")
                    headers[key] = value
                if b"transfer-encoding" in headers:
                    raise NtripError("unsupported_transfer_encoding")
                if headers.get(b"content-encoding", b"identity") != b"identity":
                    raise NtripError("unsupported_content_encoding")
                content_type = headers.get(b"content-type", b"").split(b";", 1)[0]
                if b"sourcetable" in content_type:
                    raise NtripError("sourcetable_response")
                if content_type and content_type not in {
                        b"gnss/data", b"application/octet-stream", b"application/rtcm",
                        b"application/rtcm3", b"application/gnss"}:
                    raise NtripError("unexpected_content_type")
                self.stage, self.ready = "body", True
        if self.stage == "body":
            if not self.pending:
                return b""
            if not self.pending.startswith(b"\xd3"):
                if len(self.pending) < 12:
                    return b""
                if self.pending.startswith((b"SOURCETABLE", b"STR;", b"CAS;", b"NET;")):
                    raise NtripError("sourcetable_response")
                raise NtripError("unexpected_body")
            if len(self.pending) < 3:
                return b""
            if self.pending[1] & 0xFC:
                raise NtripError("unexpected_body")
            self.body_checked = True
            result, self.pending = self.pending, b""
            return result
        return b""


class RtcmFramer:
    """Complete CRC-correct frames only; corruption drops the connection.

    Unlike an offline scanner this parser never resynchronizes past corrupt
    data, and never releases a good prefix from a feed containing a bad frame.
    The network caller reads at most 4096 bytes and queues no frame backlog.
    """
    def __init__(self, partial_timeout_s=2.0):
        self.pending = bytearray()
        self.partial_since = None
        self.partial_timeout_s = partial_timeout_s
        self.crc_failures = 0

    def check_age(self, now):
        if self.partial_since is not None and now - self.partial_since > self.partial_timeout_s:
            self.pending.clear()
            self.partial_since = None
            raise NtripError("rtcm_partial_expired")

    def feed(self, data, now=None):
        now = time.monotonic() if now is None else now
        self.check_age(now)
        if len(data) > 4096:
            raise NtripError("rtcm_chunk_too_large")
        self.pending.extend(data)
        frames = []
        while self.pending:
            if self.pending[0] != 0xD3:
                raise NtripError("rtcm_invalid_frame")
            if len(self.pending) < 3:
                break
            if self.pending[1] & 0xFC:
                raise NtripError("rtcm_invalid_frame")
            length = ((self.pending[1] & 3) << 8) | self.pending[2]
            if length < 2:
                raise NtripError("rtcm_invalid_frame")
            if len(self.pending) < length + 6:
                break
            frame = bytes(self.pending[:length + 6])
            if crc24q(frame[:-3]) != int.from_bytes(frame[-3:], "big"):
                self.crc_failures += 1
                raise NtripError("rtcm_crc_failure")
            del self.pending[:length + 6]
            frames.append(frame)
            # Any bytes remaining after this complete frame are a new partial.
            self.partial_since = None
        if self.pending and self.partial_since is None:
            self.partial_since = now
        elif not self.pending:
            self.partial_since = None
        return frames


class NtripClient(threading.Thread):
    """Read live RTCM and ask the sole serial owner for synchronous writes.

    gga_provider() -> {raw, received_mono, session} or None.
    rtcm_writer(frame) -> exact accepted byte count. It must be bounded, check
    the pause/position gate again immediately before the UART write, and never
    retry a partial write. The source owner, not this worker, holds the UART.

    A serial error/short write latches a fatal stop until this node is restarted.
    Network/CRC/stale-GGA errors close the connection and retry from new bytes.
    A pause closes the socket; resume cannot reuse pre-pause partial frames.
    """
    def __init__(self, credentials_file, gga_provider, rtcm_writer,
                 host="rtk.ntrip.qxwz.com", port=8002, mountpoint="AUTO",
                 pause_file=None, retry_s=1.0, connect_timeout_s=3.0,
                 network_idle_s=10.0, gga_period_s=10.0, frame_callback=None):
        super().__init__(name="cup_gnss_ntrip", daemon=True)
        # Validate the endpoint before any thread/socket/secret is created.
        validate_endpoint(host, port, mountpoint)
        for value in (retry_s, connect_timeout_s, network_idle_s, gga_period_s):
            if not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ValueError("NTRIP timeouts must be finite and positive")
        self.credentials_file = credentials_file
        self.gga_provider, self.rtcm_writer = gga_provider, rtcm_writer
        self.host, self.port, self.mountpoint = host, port, mountpoint
        self.pause_file = pause_file
        self.retry_s, self.connect_timeout_s = retry_s, connect_timeout_s
        self.network_idle_s, self.gga_period_s = network_idle_s, gga_period_s
        self.frame_callback = frame_callback
        self.stop_event = threading.Event()
        self._lock = threading.RLock()
        self._socket = None
        self._manual_pause = False
        self._fatal = False
        self._tracker = FreshGgaTracker()
        self._resolver = None
        self._resolve_done = threading.Event()
        self._addresses = None
        self._resolve_error = False
        self._counts = collections.Counter()
        self._received_types = collections.Counter()
        self._written_types = collections.Counter()
        self._skipped_types = collections.Counter()
        self._status = {"enabled": True, "state": "starting", "error": "",
                        "last_error": "", "fatal": False, "connected": False,
                        "gga_available": False, "gga_error": "waiting_gga",
                        "last_network_mono": None, "last_payload_mono": None,
                        "last_frame_mono": None, "last_write_mono": None,
                        "last_write_attempt_mono": None,
                        "last_gga_sent_mono": None, "last_write_error": ""}

    def _is_paused(self):
        return self._manual_pause or bool(self.pause_file and os.path.lexists(self.pause_file))

    def _update(self, **values):
        with self._lock:
            self._status.update(values)

    def _count(self, key, amount=1):
        with self._lock:
            self._counts[key] += amount

    def _gga(self):
        try:
            sample = self.gga_provider()
        except Exception:
            sample = None
            self._tracker._invalidate("gga_provider_failed")
            self._update(gga_available=False, gga_error="gga_provider_failed")
            return None
        current = self._tracker.update(sample)
        self._update(gga_available=current is not None, gga_error=self._tracker.error)
        return current

    def snapshot(self):
        now = time.monotonic()
        with self._lock:
            result = dict(self._status)
            result.update({key: self._counts[key] for key in (
                "connection_attempts", "connections", "reconnects", "network_bytes",
                "rtcm_payload_bytes", "received_frames", "received_bytes",
                "write_attempts", "write_attempted_bytes", "written_frames", "written_bytes",
                "gga_sent", "crc_failures", "short_writes", "serial_errors",
                "write_unknown_count", "callback_errors")})
            result.update(paused=self._is_paused(),
                          allowed_types=sorted(ALLOWED_RTCM_IDS),
                          received_types=dict(self._received_types),
                          written_types=dict(self._written_types),
                          skipped_types=dict(self._skipped_types))
        for label in ("network", "payload", "frame", "write", "write_attempt", "gga_sent"):
            stamp = result["last_" + label + "_mono"]
            result[label + "_age_s"] = None if stamp is None else max(0.0, now - stamp)
        return result

    def _close_socket(self):
        with self._lock:
            sock, self._socket = self._socket, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        self._update(connected=False)

    def stop(self):
        self.stop_event.set()
        self._close_socket()

    def pause(self):
        self._manual_pause = True
        self._close_socket()

    def resume(self):
        # A file pause is intentional and is removed by the workflow command.
        self._manual_pause = False

    def _check_gate(self):
        if self.stop_event.is_set():
            raise NtripError("stopped")
        if self._is_paused():
            raise NtripError("paused")
        current = self._gga()
        if current is None:
            raise NtripError("gga_expired")
        return current

    def _resolve(self):
        # getaddrinfo has no Python timeout. One daemon helper per client keeps
        # stop()/join() bounded even during a stuck resolver, without spawning
        # repeated unresolved DNS tasks on retries. No credentials enter it.
        if self._resolver is None:
            def resolve():
                try:
                    self._addresses = socket.getaddrinfo(self.host, self.port,
                                                         type=socket.SOCK_STREAM)
                except OSError:
                    self._resolve_error = True
                finally:
                    self._resolve_done.set()
            self._resolver = threading.Thread(target=resolve, name="cup_gnss_ntrip_dns", daemon=True)
            self._resolver.start()
        deadline = time.monotonic() + self.connect_timeout_s
        while not self._resolve_done.wait(0.05):
            self._check_gate()
            if time.monotonic() >= deadline:
                raise NtripError("network_resolve_timeout")
        if self._resolve_error or not self._addresses:
            self._resolver = None
            self._resolve_done.clear()
            self._resolve_error = False
            raise NtripError("network_resolve_failed")
        return self._addresses

    def _connect(self):
        addresses = self._resolve()
        deadline = time.monotonic() + self.connect_timeout_s
        for family, kind, protocol, _, address in addresses:
            self._check_gate()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            sock = socket.socket(family, kind, protocol)
            with self._lock:
                self._socket = sock
            try:
                sock.settimeout(remaining)
                sock.connect(address)
                sock.settimeout(0.2)
                return sock
            except OSError:
                self._close_socket()
        raise NtripError("network_connect_failed")

    def _frame_event(self, direction, frame, message_id, now, count=None, error=""):
        if self.frame_callback is not None:
            try:
                self.frame_callback({"direction": direction, "frame": frame,
                                     "message_id": message_id, "monotonic_s": now,
                                     "accepted_count": count, "error": error})
            except Exception:
                self._count("callback_errors")
                # Evidence loss must be visible, but does not send UART bytes.

    def _forward(self, payload, now, framer):
        self._count("rtcm_payload_bytes", len(payload))
        self._update(last_payload_mono=now)
        before = framer.crc_failures
        try:
            frames = framer.feed(payload, now)
        finally:
            self._count("crc_failures", framer.crc_failures - before)
        for frame in frames:
            message_id = (frame[3] << 4) | (frame[4] >> 4)
            with self._lock:
                self._received_types[str(message_id)] += 1
            self._count("received_frames")
            self._count("received_bytes", len(frame))
            self._update(last_frame_mono=now)
            self._frame_event("rx", frame, message_id, now)
            self._check_gate()
            if time.monotonic() - now > 2.0:
                raise NtripError("rtcm_delivery_expired")
            if message_id not in ALLOWED_RTCM_IDS:
                with self._lock:
                    self._skipped_types[str(message_id)] += 1
                continue
            self._count("write_attempts")
            self._count("write_attempted_bytes", len(frame))
            self._update(last_write_attempt_mono=time.monotonic())
            try:
                count = self.rtcm_writer(frame)
            except ForwardingBlocked:
                self._update(last_write_error="forwarding_gate_closed")
                self._frame_event("tx", frame, message_id, time.monotonic(), 0, "forwarding_gate_closed")
                raise NtripError("forwarding_gate_closed") from None
            except Exception:
                self._fatal = True
                self._count("serial_errors")
                self._count("write_unknown_count")
                self._update(fatal=True, last_write_error="serial_write_failed")
                self._frame_event("tx", frame, message_id, time.monotonic(), error="serial_write_failed")
                raise NtripError("serial_write_failed") from None
            valid_count = isinstance(count, int) and not isinstance(count, bool) and 0 <= count <= len(frame)
            if valid_count:
                self._count("written_bytes", count)
            else:
                self._count("write_unknown_count")
            error = "" if valid_count and count == len(frame) else "serial_partial_write"
            self._frame_event("tx", frame, message_id, time.monotonic(), count if valid_count else None, error)
            if error:
                self._fatal = True
                self._count("short_writes")
                self._update(fatal=True, last_write_error=error)
                raise NtripError(error)
            self._count("written_frames")
            with self._lock:
                self._written_types[str(message_id)] += 1
            self._update(last_write_mono=time.monotonic(), last_write_error="")

    def _session(self, request):
        self._count("connection_attempts")
        self._update(state="connecting", error="")
        sock = self._connect()
        self._check_gate()
        sock.sendall(request)
        self._count("connections")
        self._update(connected=True, state="streaming")
        response, framer = NtripResponseParser(), RtcmFramer()
        started = last_bytes = last_frame = time.monotonic()
        last_gga_sent = None
        while not self.stop_event.is_set():
            current = self._check_gate()
            now = time.monotonic()
            # Send GGA after accepted status even when a VRS caster waits for
            # it before sending optional headers or any RTCM bytes.
            if response.status == 200 and (last_gga_sent is None or now - last_gga_sent >= self.gga_period_s):
                sock.sendall(current)
                last_gga_sent = time.monotonic()
                self._count("gga_sent")
                self._update(last_gga_sent_mono=last_gga_sent)
            if not response.ready and now - started > 10.0:
                raise NtripError("header_timeout")
            if now - last_bytes > self.network_idle_s or now - last_frame > self.network_idle_s:
                raise NtripError("network_rtcm_timeout")
            framer.check_age(now)
            try:
                data = sock.recv(4096)
            except socket.timeout:
                continue
            if not data:
                raise NtripError("network_closed")
            last_bytes = time.monotonic()
            self._count("network_bytes", len(data))
            self._update(last_network_mono=last_bytes)
            payload = response.feed(data)
            if payload:
                before = self._counts["received_frames"]
                self._forward(payload, last_bytes, framer)
                if self._counts["received_frames"] > before:
                    last_frame = time.monotonic()

    def run(self):
        try:
            while not self.stop_event.is_set():
                self._gga()
                if self._fatal:
                    self._update(state="fatal", fatal=True)
                    self.stop_event.wait(0.1)
                    continue
                if self._is_paused():
                    self._update(state="paused", error="")
                    self.stop_event.wait(0.1)
                    continue
                if not self._status["gga_available"]:
                    self._update(state="waiting_gga", error=self._status["gga_error"])
                    self.stop_event.wait(0.1)
                    continue
                try:
                    username, password = load_credentials(self.credentials_file)
                    request = build_request(self.host, self.port, self.mountpoint, username, password)
                    username = password = None
                    self._session(request)
                    request = None
                except NtripError as exc:
                    reason = str(exc)
                    self._update(state="fatal" if self._fatal else "retrying", error=reason, last_error=reason)
                except OSError:
                    self._update(state="retrying", error="network_io_failed", last_error="network_io_failed")
                except Exception:
                    # Callback exceptions and library messages must not expose
                    # credentials, request headers, or arbitrary exception text.
                    self._update(state="retrying", error="ntrip_internal_error", last_error="ntrip_internal_error")
                finally:
                    request = None
                    self._close_socket()
                self._count("reconnects")
                self.stop_event.wait(self.retry_s)
        finally:
            self._close_socket()
            self._update(state="stopped")
