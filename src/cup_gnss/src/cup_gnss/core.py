"""Released NMEA parser with bounded, conservative rover GNSS state.

Timing uses a local monotonic clock. TCP capture wall timestamps are checked
against the receiving host's wall clock, never silently replaced with now().
GGA time-of-day is only an ordering/freshness signal, not a dated ROS stamp.
"""
import collections
import math
import re
import time


QUALITY_NAMES = {
    0: "invalid", 1: "single", 2: "differential", 3: "PPS",
    4: "RTK fixed", 5: "RTK float", 6: "estimated/dead reckoning",
    7: "manual", 8: "simulation",
}
SERVICE_BY_TALKER = {"GP": 1, "GL": 2, "GA": 8, "GB": 4, "BD": 4}


def finite_number(value):
    if isinstance(value, bool):
        return None
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (TypeError, ValueError, OverflowError):
        return None


def nonnegative(value):
    value = finite_number(value)
    return value if value is not None and value >= 0 else None


def integer(value):
    return int(value) if isinstance(value, str) and re.fullmatch(r"\d+", value) else None


def coordinate(value, hemisphere, latitude):
    if hemisphere not in (("N", "S") if latitude else ("E", "W")):
        return None
    digits = 2 if latitude else 3
    if not re.fullmatch(r"\d{" + str(digits + 2) + r"}(?:\.\d+)?", value):
        return None
    degrees, minutes = int(value[:digits]), float(value[digits:])
    limit = 90 if latitude else 180
    if minutes >= 60 or degrees > limit or (degrees == limit and minutes != 0):
        return None
    result = degrees + minutes / 60.0
    return -result if hemisphere in ("S", "W") else result


def time_of_day(value):
    if not re.fullmatch(r"\d{6}(?:\.\d+)?", value):
        return None
    hours, minutes, seconds = int(value[:2]), int(value[2:4]), float(value[4:])
    if hours >= 24 or minutes >= 60 or seconds >= 60:
        return None
    return hours * 3600 + minutes * 60 + seconds


# The default parser is the locked released ROS-driver library. The remaining
# small numeric/time guards serve project freshness and evidence contracts.
from .upstream_nmea import parse_nmea, BACKEND as NMEA_BACKEND


class GnssModel:
    """Only advancing, fresh GGA can cause a valid position publication.

    All public methods are called by the ROS main thread; source workers only
    enqueue records. A new serial session or TCP connection requires two
    advancing GGA epochs, even if the first sentence claims a valid fix.
    """
    def __init__(self, source="tcp", data_timeout_s=2.5, max_differential_age_s=5.0,
                 max_source_age_s=2.0, future_tolerance_s=0.5):
        for value in (data_timeout_s, max_differential_age_s, max_source_age_s):
            if finite_number(value) is None or value <= 0:
                raise ValueError("timeouts and age limits must be positive and finite")
        if finite_number(future_tolerance_s) is None or future_tolerance_s < 0:
            raise ValueError("future_tolerance_s must be nonnegative and finite")
        self.source = source
        self.data_timeout_s = float(data_timeout_s)
        self.max_differential_age_s = float(max_differential_age_s)
        self.max_source_age_s = float(max_source_age_s)
        self.future_tolerance_s = float(future_tolerance_s)
        self.transport_connected = False
        self.serial_connected = False
        self.session = None
        self.last_error = "waiting for source"
        self.source_fault = ""
        self.port = ""
        self.counts = collections.Counter()
        self.nmea_times = collections.deque(maxlen=4096)
        self.gga_times = collections.deque(maxlen=4096)
        self._reset_stream()

    def _reset_stream(self):
        self.seq = -1
        self.latest_gga = None
        self.latest_gga_raw = None
        self.latest_rmc = None
        self.latest_gst = None
        self.last_nmea = None
        self.last_gga_receipt = None
        self.last_epoch = None
        self.last_advance_mono = None
        self.advancing_epochs = 0
        self.source_age_s = None
        self.nmea_times.clear()
        self.gga_times.clear()

    def _switch_session(self, session):
        if session != self.session:
            self._reset_stream()
            self.session = session
            self.source_fault = ""

    def _reject(self, reason):
        self.counts["rejected_source_records"] += 1
        self.source_fault = reason
        self.last_error = reason
        self.latest_gga = None
        return {"raw": None, "publish_fix": False, "changed": True}

    @staticmethod
    def _valid_session(value):
        return isinstance(value, str) and 1 <= len(value) <= 128 and value.isprintable()

    def ingest(self, event, now_wall=None, now_mono=None):
        """Consume one live source event, returning publication instructions."""
        now_wall = time.time() if now_wall is None else now_wall
        now_mono = time.monotonic() if now_mono is None else now_mono
        result = {"raw": None, "publish_fix": False, "changed": False}
        if not isinstance(event, dict):
            return self._reject("malformed_source_record")
        kind = event.get("kind")
        if kind == "transport":
            self.transport_connected = event.get("connected") is True
            self.serial_connected = False
            self.session = None
            self._reset_stream()
            self.source_fault = ""
            self.last_error = str(event.get("error", ""))[:160]
            result["changed"] = True
            return result
        if event.get("v") != 1 or isinstance(event.get("v"), bool) or kind not in ("nmea", "status"):
            return self._reject("unsupported_source_record")
        if not self.transport_connected:
            return self._reject("record_while_transport_disconnected")
        session = event.get("session")
        if not self._valid_session(session):
            return self._reject("invalid_serial_session")
        rx_mono = finite_number(event.get("_received_mono", now_mono))
        rx_wall = finite_number(event.get("_received_wall", now_wall))
        if rx_mono is None or rx_wall is None or rx_mono > now_mono + 0.01:
            return self._reject("invalid_receipt_timestamp")
        capture = finite_number(event.get("captured_at" if kind == "nmea" else "sent_at"))
        if capture is None:
            return self._reject("missing_source_timestamp")
        source_age = rx_wall - capture
        if source_age < -self.future_tolerance_s:
            return self._reject("source_clock_ahead")
        if max(0.0, source_age) + max(0.0, now_mono - rx_mono) > self.max_source_age_s:
            return self._reject("source_record_too_old")
        self._switch_session(session)
        self.source_age_s = source_age
        if kind == "status":
            if not isinstance(event.get("connected"), bool):
                return self._reject("invalid_serial_status")
            connected = event["connected"]
            if not connected or not self.serial_connected:
                self._reset_stream()
            self.serial_connected = connected
            self.port = str(event.get("port", ""))[:256]
            self.last_error = str(event.get("error") or "")[:160]
            if not connected:
                self.source_fault = ""
            result["changed"] = True
            return result
        seq = event.get("seq")
        if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
            return self._reject("invalid_source_sequence")
        if seq <= self.seq:
            return self._reject("duplicate_or_reordered_source_sequence")
        self.seq = seq
        raw = event.get("nmea")
        if not isinstance(raw, str) or len(raw) > 1024 or not raw.isascii() or "\n" in raw or "\r" in raw:
            return self._reject("invalid_raw_nmea_record")
        self.serial_connected = True
        self.source_fault = ""
        self.last_error = ""
        result["raw"] = raw
        parsed = parse_nmea(raw)
        if parsed is None:
            self.counts["checksum_errors"] += 1
            return result
        self.counts["nmea_count"] += 1
        self.last_nmea = (rx_mono, max(0.0, source_age))
        self.nmea_times.append(rx_mono)
        if parsed["kind"] == "RMC":
            self.latest_rmc = parsed
        if parsed["kind"] == "GST":
            self.latest_gst = parsed
        if parsed["kind"] != "GGA":
            return result
        self.counts["gga_count"] += 1
        self.gga_times.append(rx_mono)
        self.latest_gga_raw = parsed
        self.last_gga_receipt = (rx_mono, max(0.0, source_age))
        epoch = parsed["epoch"]
        if epoch is None:
            self.latest_gga = None
            self.last_error = "invalid_gga_time" if parsed["quality"] in (1, 2, 3, 4, 5) else ""
            result["publish_fix"] = True
            return result
        delta = None if self.last_epoch is None else (epoch - self.last_epoch) % 86400
        if delta is not None and (delta == 0 or delta >= 43200):
            self.counts["duplicate_epochs" if delta == 0 else "older_epochs"] += 1
            # Even a repeated timestamp can report loss of fix or old correction
            # age. It must never leave earlier coordinates published as valid.
            self.latest_gga = None
            self.last_error = "duplicate_gga_epoch" if delta == 0 else "older_gga_epoch"
            result["publish_fix"] = True
            return result
        self.last_epoch = epoch
        self.last_advance_mono = rx_mono
        self.advancing_epochs += 1
        self.latest_gga = (parsed, rx_mono, max(0.0, source_age))
        result["publish_fix"] = True
        return result

    @staticmethod
    def _age(sample, now_mono):
        return None if sample is None else max(0.0, now_mono - sample[0]) + sample[1]

    @staticmethod
    def _rate(times, now_mono):
        while times and times[0] < now_mono - 10.0:
            times.popleft()
        if len(times) < 2 or times[-1] <= times[0]:
            return 0.0
        return (len(times) - 1) / (times[-1] - times[0])

    def snapshot(self, now_mono=None):
        now_mono = time.monotonic() if now_mono is None else now_mono
        raw = self.latest_gga_raw or {}
        record = self.latest_gga[0] if self.latest_gga else {}
        gga_age = self._age(self.last_gga_receipt, now_mono)
        fix_epoch_age = None if self.latest_gga is None else self._age(self.latest_gga[1:], now_mono)
        data_age = self._age(self.last_nmea, now_mono)
        advance_age = None if self.last_advance_mono is None else max(0.0, now_mono - self.last_advance_mono)
        quality = raw.get("quality")
        diff_age = raw.get("differential_age_s")
        # Differential age increases between GGA epochs. It is checked independently
        # of the receiver's quality field, which can remain fixed after RTCM stops.
        effective_diff_age = None if diff_age is None else diff_age + (gga_age or 0.0)
        diff_ok = effective_diff_age is not None and effective_diff_age <= self.max_differential_age_s
        reason, level = "", 0
        if not self.transport_connected:
            reason, level = "transport_disconnected", 2
        elif not self.serial_connected:
            reason, level = "serial_disconnected", 2
        elif self.source_fault:
            reason, level = self.source_fault, 2
        elif data_age is None or data_age > self.data_timeout_s:
            reason, level = "nmea_timeout", 2
        elif gga_age is None or gga_age > self.data_timeout_s:
            reason, level = "gga_timeout", 2
        elif quality not in (1, 2, 3, 4, 5):
            # A receiver indoors can report fresh GGA quality=0 with blank UTC.
            # That is an operational no-fix state, not a serial/GGA timeout.
            reason, level = "receiver_has_no_measured_fix", 1
        elif raw.get("epoch") is None:
            reason, level = "invalid_gga_time", 1
        elif self.latest_gga is None:
            reason, level = "gnss_time_not_advancing", 2
        elif fix_epoch_age is None or fix_epoch_age > self.data_timeout_s:
            reason, level = "fix_epoch_timeout", 2
        elif advance_age is None or advance_age > self.data_timeout_s:
            reason, level = "gnss_time_not_advancing", 2
        elif self.advancing_epochs < 2:
            reason, level = "awaiting_advancing_gga", 1
        elif record.get("latitude") is None or record.get("longitude") is None:
            reason, level = "missing_or_invalid_coordinates", 1
        elif quality in (2, 4, 5) and not diff_ok:
            reason, level = "differential_age_missing_or_stale", 2
        valid = not reason
        if valid and quality != 4:
            level = 1
        fix = {"valid": valid, "status": (2 if quality in (2, 4, 5) else 0) if valid else -1,
               "service": SERVICE_BY_TALKER.get(record.get("talker"), 0),
               "latitude": record["latitude"] if valid else math.nan,
               "longitude": record["longitude"] if valid else math.nan,
               "altitude": record.get("altitude") if valid and record.get("altitude") is not None else math.nan,
               "position_covariance": [0.0] * 9, "position_covariance_type": 0}
        if reason:
            message = reason
        else:
            message = QUALITY_NAMES.get(quality, "unknown")
        rmc = self.latest_rmc or {}
        gst = self.latest_gst or {}
        errors = gst.get("receiver_error_fields", {})
        diagnostics = {
            "nmea_backend": NMEA_BACKEND,
            "receiver_uncertainty_source": "receiver GST; not time-associated to GGA" if gst else "none",
            "receiver_gst_utc_time": gst.get("utc_time", ""),
            "receiver_gst_lat_std_m": errors.get("lat_std_dev"),
            "receiver_gst_lon_std_m": errors.get("lon_std_dev"),
            "receiver_gst_alt_std_m": errors.get("alt_std_dev"),
            "source": self.source, "transport_connected": self.transport_connected,
            "serial_connected": self.serial_connected, "session": self.session or "",
            "port": self.port, "receiver_status": message,
            "nmea_hz": self._rate(self.nmea_times, now_mono),
            "gga_hz": self._rate(self.gga_times, now_mono),
            "gga_quality": raw.get("quality"), "fix_type": QUALITY_NAMES.get(raw.get("quality"), "unknown"),
            "satellites": raw.get("satellites"), "hdop": raw.get("hdop"),
            "differential_age_s": raw.get("differential_age_s"),
            "effective_differential_age_s": effective_diff_age,
            "differential_age_ok": diff_ok if raw.get("quality") in (2, 4, 5) else None,
            "max_differential_age_s": self.max_differential_age_s,
            "data_age_s": data_age, "gga_age_s": gga_age,
            "fix_epoch_age_s": fix_epoch_age,
            "source_age_s": self.source_age_s,
            "gnss_time_advance_age_s": advance_age, "gga_utc_time": raw.get("utc_time", ""),
            "fix_valid": valid, "invalid_reason": reason,
            "nmea_count": self.counts["nmea_count"], "gga_count": self.counts["gga_count"],
            "checksum_errors": self.counts["checksum_errors"],
            "duplicate_epochs": self.counts["duplicate_epochs"], "older_epochs": self.counts["older_epochs"],
            "rejected_source_records": self.counts["rejected_source_records"],
            "last_error": self.last_error, "rmc_status": rmc.get("status", ""),
            "rmc_mode": rmc.get("mode", ""), "rmc_navigation_status": rmc.get("navigation_status", ""),
            "covariance": "UNKNOWN", "position_stamp": "ROS receipt time; freshness uses monotonic + source wall age",
        }
        return {"fix": fix, "level": level, "message": message, "diagnostics": diagnostics}
