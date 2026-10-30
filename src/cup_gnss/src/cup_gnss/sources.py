"""Live TCP/serial sources; one serial owner reads NMEA and writes validated RTCM.

The serial subclass preserves serial_probe.py's POSIX no-input-flush hook,
captures original termios before reconfiguration, and restores it on close.
Opening/selecting a UART still makes OS/USB control requests. Requested-low
DTR/RTS can briefly transition in a driver; this is not a zero-control open.
"""
import copy
import json
import os
import queue
import socket
import threading
import time
import uuid

from cup_gnss.core import parse_nmea
from cup_gnss.ntrip import ForwardingBlocked, FreshGgaTracker


class SourceProtocolError(Exception):
    pass


class LineFramer:
    """Bounded serial line framing; overlong/binary lines are discarded whole."""
    def __init__(self, limit=1024):
        self.limit = limit
        self.pending = bytearray()
        self.discarding = False

    def feed(self, chunk):
        lines = []
        for byte in chunk:
            if byte in (10, 13):
                if self.pending and not self.discarding:
                    try:
                        line = self.pending.decode("ascii")
                    except UnicodeDecodeError:
                        line = ""
                    if line.startswith(("$", "!")):
                        lines.append(line)
                self.pending.clear()
                self.discarding = False
            elif not self.discarding:
                self.pending.append(byte)
                if len(self.pending) > self.limit:
                    self.pending.clear()
                    self.discarding = True
        return lines


def make_receiver_class(serial_module):
    """Extracted, independent copy of the proven serial_probe.py hook."""
    class RestorableSerial(serial_module.Serial):
        original_termios = None

        def _reset_input_buffer(self):
            return None

        def _reconfigure_port(self, *args, **kwargs):
            if self.original_termios is None and os.name == "posix":
                import termios
                self.original_termios = copy.deepcopy(termios.tcgetattr(self.fd))
            return super()._reconfigure_port(*args, **kwargs)
    return RestorableSerial


class LiveSource(threading.Thread):
    def __init__(self, events, retry_s=1.0):
        super().__init__(name="cup_gnss_live_source", daemon=True)
        self.events = events
        self.retry_s = retry_s
        self.stop_event = threading.Event()
        self.active_socket = None

    def emit(self, event):
        event = dict(event)
        event["_received_wall"] = time.time()
        event["_received_mono"] = time.monotonic()
        try:
            self.events.put_nowait(event)
        except queue.Full:
            # Never allow a backlog to be replayed as live data after a stall.
            while True:
                try:
                    self.events.get_nowait()
                except queue.Empty:
                    break
            self.events.put_nowait({"kind": "transport", "connected": False,
                                   "error": "receive_queue_overflow",
                                   "_received_wall": time.time(),
                                   "_received_mono": time.monotonic()})
            raise SourceProtocolError("receive_queue_overflow")

    def stop(self):
        self.stop_event.set()
        sock = self.active_socket
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def connect_once(self):
        raise NotImplementedError

    def run(self):
        while not self.stop_event.is_set():
            try:
                self.connect_once()
                error = "source_closed"
            except SourceProtocolError as exc:
                error = str(exc)
            except (OSError, ValueError, ImportError) as exc:
                error = type(exc).__name__
            except Exception as exc:
                # Keep source failures visible without crashing the diagnostic loop.
                error = "source_error:" + type(exc).__name__
            try:
                self.emit({"kind": "transport", "connected": False, "error": error})
            except SourceProtocolError:
                pass
            if self.stop_event.wait(self.retry_s):
                return


class TcpSource(LiveSource):
    def __init__(self, events, host="host.docker.internal", port=10110,
                 connect_timeout_s=2.0, idle_timeout_s=3.0, **kwargs):
        super().__init__(events, **kwargs)
        self.host, self.port = host, port
        self.connect_timeout_s = connect_timeout_s
        self.idle_timeout_s = idle_timeout_s

    def connect_once(self):
        pending = bytearray()
        with socket.create_connection((self.host, self.port), self.connect_timeout_s) as sock:
            self.active_socket = sock
            sock.settimeout(0.2)
            self.emit({"kind": "transport", "connected": True, "error": ""})
            last_bytes = time.monotonic()
            try:
                while not self.stop_event.is_set():
                    try:
                        chunk = sock.recv(4096)
                    except socket.timeout:
                        if time.monotonic() - last_bytes > self.idle_timeout_s:
                            raise SourceProtocolError("tcp_heartbeat_timeout")
                        continue
                    if not chunk:
                        return
                    last_bytes = time.monotonic()
                    pending.extend(chunk)
                    while b"\n" in pending:
                        ending = pending.index(10)
                        if ending > 4096:
                            raise SourceProtocolError("tcp_record_too_large")
                        line = bytes(pending[:ending])
                        del pending[:ending + 1]
                        try:
                            event = json.loads(line.decode("utf-8"))
                        except (UnicodeError, ValueError):
                            raise SourceProtocolError("invalid_tcp_json")
                        if not isinstance(event, dict) or event.get("kind") not in ("status", "nmea"):
                            raise SourceProtocolError("invalid_tcp_record_kind")
                        # Do not trust wire-supplied local-receipt metadata.
                        event.pop("_received_mono", None)
                        event.pop("_received_wall", None)
                        self.emit(event)
                    if len(pending) > 4096:
                        raise SourceProtocolError("tcp_record_too_large")
            finally:
                self.active_socket = None


class SerialSource(LiveSource):
    def __init__(self, events, port, baud=115200, pause_file=None, expected_serial="", **kwargs):
        super().__init__(events, **kwargs)
        self.port, self.baud = port, baud
        self.pause_file = pause_file
        self.expected_serial = expected_serial
        self.write_requests = queue.Queue(maxsize=1)
        self.gga_lock = threading.Lock()
        self.current_gga = None
        self.write_gga_tracker = FreshGgaTracker()
        self.last_rtcm_write_mono = None
        self.last_rtcm_write_session = ""

    def get_gga(self):
        with self.gga_lock:
            return dict(self.current_gga) if self.current_gga else None

    def rtcm_write_snapshot(self):
        with self.gga_lock:
            stamp = self.last_rtcm_write_mono
            return {"rtcm_serial_write_age_s": None if stamp is None else max(0.0, time.monotonic() - stamp),
                    "rtcm_serial_write_session": self.last_rtcm_write_session}

    def write_rtcm(self, frame):
        """Called by NTRIP; the serial thread alone executes receiver.write()."""
        gga = self.get_gga()
        if not gga or self.stop_event.is_set():
            raise ForwardingBlocked("serial_not_ready")
        request = {"frame": bytes(frame), "session": gga["session"],
                   "deadline": time.monotonic() + 0.8, "done": threading.Event(),
                   "cancelled": False, "result": None, "error": None}
        try:
            self.write_requests.put_nowait(request)
        except queue.Full:
            raise ForwardingBlocked("serial_write_queue_busy")
        if not request["done"].wait(1.0):
            request["cancelled"] = True
            raise SourceProtocolError("serial_write_ack_timeout")
        if request["error"]:
            raise request["error"]
        return request["result"]

    def _check_identity(self):
        if not self.expected_serial:
            return
        from pathlib import Path
        st = os.stat(self.port)
        parent = Path("/sys/dev/char/{}:{}".format(os.major(st.st_rdev), os.minor(st.st_rdev))).resolve()
        for candidate in (parent,) + tuple(parent.parents):
            serial_path = candidate / "serial"
            if serial_path.is_file():
                if (serial_path.read_text().strip() != self.expected_serial
                        or (candidate / "idVendor").read_text().strip() != "1a86"
                        or (candidate / "idProduct").read_text().strip() != "55d3"):
                    raise SourceProtocolError("usb_identity_mismatch")
                return
        raise SourceProtocolError("usb_identity_unavailable")

    def _service_writes(self, receiver, session):
        try:
            request = self.write_requests.get_nowait()
        except queue.Empty:
            return
        try:
            gga = self.get_gga()
            now = time.monotonic()
            if self.stop_event.is_set():
                raise ForwardingBlocked("serial_stopping")
            if request["cancelled"] or now > request["deadline"]:
                raise ForwardingBlocked("serial_write_request_expired")
            if self.pause_file and os.path.lexists(self.pause_file):
                raise ForwardingBlocked("corrections_paused")
            if not gga or gga["session"] != session or request["session"] != session:
                raise ForwardingBlocked("serial_session_changed")
            parsed = parse_nmea(gga["raw"])
            if (now - gga["received_mono"] > 2.5 or not parsed
                    or parsed.get("quality") not in (1, 2, 3, 4, 5)
                    or parsed.get("latitude") is None or parsed.get("longitude") is None):
                raise ForwardingBlocked("serial_gga_not_fresh")
            if self.write_gga_tracker.update(gga) is None:
                raise ForwardingBlocked("serial_gga_not_advancing")
            request["result"] = receiver.write(request["frame"])
            if request["result"] == len(request["frame"]):
                with self.gga_lock:
                    self.last_rtcm_write_mono = time.monotonic()
                    self.last_rtcm_write_session = session
            # Return a short count intact; NTRIP records it and latches fatal.

        except Exception as exc:
            request["error"] = exc if isinstance(exc, ForwardingBlocked) else SourceProtocolError("serial_write_" + type(exc).__name__)
        finally:
            request["done"].set()

    def _clear_pending_writes(self):
        with self.gga_lock:
            self.current_gga = None
            self.last_rtcm_write_mono = None
            self.last_rtcm_write_session = ""
        while True:
            try:
                request = self.write_requests.get_nowait()
            except queue.Empty:
                break
            request["error"] = ForwardingBlocked("serial_disconnected")
            request["done"].set()

    def connect_once(self):
        self._clear_pending_writes()
        self._check_identity()
        import serial  # Core/parser/TCP operation does not require pyserial import.
        receiver = make_receiver_class(serial)(
            port=None, baudrate=self.baud, bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=0.2, write_timeout=0.5, xonxoff=False, rtscts=False,
            dsrdtr=False, exclusive=True if os.name == "posix" else None,
        )
        receiver.dtr = False
        receiver.rts = False
        receiver.port = self.port
        session, seq, last_rx_at = str(uuid.uuid4()), 0, None
        exclusive = False
        framer = LineFramer()

        def status(connected, error=""):
            self.emit({"v": 1, "kind": "status", "session": session,
                       "connected": connected, "last_rx_at": last_rx_at,
                       "sent_at": time.time(), "port": self.port, "error": error})
        try:
            receiver.open()
            if os.name == "posix":
                import fcntl
                import termios
                if not hasattr(termios, "TIOCEXCL"):
                    raise SourceProtocolError("serial_exclusive_ioctl_unavailable")
                fcntl.ioctl(receiver.fileno(), termios.TIOCEXCL)
                exclusive = True
            self.emit({"kind": "transport", "connected": True, "error": ""})
            status(True)
            next_status = time.monotonic() + 1.0
            while not self.stop_event.is_set():
                self._service_writes(receiver, session)
                chunk = receiver.read(min(max(receiver.in_waiting, 1), 4096))
                captured_at = time.time()
                if chunk:
                    last_rx_at = captured_at
                    for raw in framer.feed(chunk):
                        parsed = parse_nmea(raw)
                        if parsed and parsed.get("kind") == "GGA":
                            with self.gga_lock:
                                self.current_gga = {"raw": raw, "received_mono": time.monotonic(), "session": session}
                            self.write_gga_tracker.update(self.get_gga())
                        seq += 1
                        self.emit({"v": 1, "kind": "nmea", "session": session,
                                   "seq": seq, "captured_at": captured_at, "nmea": raw})
                if time.monotonic() >= next_status:
                    status(True)
                    next_status = time.monotonic() + 1.0
        finally:
            self._clear_pending_writes()
            if receiver.is_open:
                if receiver.original_termios is not None:
                    try:
                        import termios
                        termios.tcsetattr(receiver.fileno(), termios.TCSANOW, receiver.original_termios)
                    except (OSError, termios.error):
                        # Device removal may prevent restoration. It is reported
                        # through the connection failure; no flushing is attempted.
                        pass
                if exclusive:
                    try:
                        import fcntl
                        import termios
                        fcntl.ioctl(receiver.fileno(), termios.TIOCNXCL)
                    except OSError:
                        pass
                receiver.close()
