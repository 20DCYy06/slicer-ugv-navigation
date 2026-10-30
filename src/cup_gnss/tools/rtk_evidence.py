#!/usr/bin/env python3
"""Actual ROS GNSS capture and conservative fixed-origin ENU evidence.

Run capture inside the Jetson Noetic environment. This tool never opens serial,
connects to NTRIP, publishes commands, or moves a robot. Offline analyze needs
only Python 3.8+. Quality=4 and fresh RTK are reported separately. Relative ENU
motion is not a measurement of absolute positioning accuracy.
"""
import argparse
import bisect
import collections
import csv
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import socket
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from cup_gnss.core import parse_nmea, time_of_day
from cup_gnss.enu import FixedOriginTrack


GNSS_TOPICS = {"/gnss/nmea": "nmea", "/gnss/fix": "fix", "/gnss/diagnostics": "diagnostics"}
VEHICLE_TOPICS = ("/Odometry", "/odom", "/tf", "/tf_static")


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def finite(value):
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (ValueError, TypeError, OverflowError):
        return None


def json_write(path, value):
    with open(path, "x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")


def ros_dict(value, nonfinite=None, path=""):
    """Serialize all ROS fields; retain NaN/Inf identity in a separate map."""
    nonfinite = {} if nonfinite is None else nonfinite
    if isinstance(value, float) and not math.isfinite(value):
        nonfinite[path] = "nan" if math.isnan(value) else ("+inf" if value > 0 else "-inf")
        return None
    if hasattr(value, "__slots__"):
        return {key: ros_dict(getattr(value, key), nonfinite, path + "." + key) for key in value.__slots__}
    if isinstance(value, (list, tuple)):
        return [ros_dict(item, nonfinite, path + "[{}]".format(index)) for index, item in enumerate(value)]
    if isinstance(value, bytes):
        return list(value)
    return value


def inspect_nmea(raw):
    parsed = parse_nmea(raw)
    gga = None
    if parsed is not None and parsed["kind"] == "GGA":
        gga = {"utc": parsed["utc_time"], "quality": parsed["quality"],
               "latitude": parsed["latitude"], "longitude": parsed["longitude"],
               "altitude_ellipsoid_m": parsed["altitude"], "altitude_msl_m": parsed["altitude_msl"],
               "geoid_separation_m": parsed["geoid_separation"], "satellites": parsed["satellites"],
               "hdop": parsed["hdop"], "differential_age_s": parsed["differential_age_s"]}
    return {"checksum_valid": parsed is not None, "gga": gga}


def diagnostic_values(record):
    if record:
        for status in record.get("statuses", []):
            if status.get("name") == "cup_gnss/receiver":
                return status.get("values", {})
    return {}


def qualify(gga, fix, diagnostic, advancing, max_age=5.0, timeout=2.5, diagnostic_age_s=0.0):
    """Independent raw/ROS checks plus explicit current node freshness policy."""
    if gga.get("quality") != 4:
        return "raw_quality_not_rtk_fixed"
    if not advancing:
        return "gga_epoch_not_advancing"
    if any(finite(gga.get(key)) is None for key in ("latitude", "longitude", "altitude_ellipsoid_m")):
        return "raw_coordinates_or_ellipsoid_height_missing"
    raw_age = finite(gga.get("differential_age_s"))
    if raw_age is None or not 0 <= raw_age <= max_age:
        return "raw_differential_age_missing_or_stale"
    if fix is None:
        return "no_nearby_ros_fix"
    if fix.get("status") != 2:
        return "ros_fix_not_augmented_valid"
    for key, raw_key, tolerance in (("latitude", "latitude", 1e-7), ("longitude", "longitude", 1e-7),
                                     ("altitude", "altitude_ellipsoid_m", 0.01)):
        value = finite(fix.get(key))
        if value is None or abs(value - gga[raw_key]) > tolerance:
            return "ros_raw_position_mismatch"
    values = diagnostic_values(diagnostic)
    if not values:
        return "no_nearby_receiver_diagnostics"
    # Separate ROS subscription callbacks can have different queue delays.
    # Matching static coordinates and receipt proximity cannot authorize an
    # earlier invalid epoch using a later recovered diagnostic snapshot.
    raw_epoch = time_of_day(gga.get("utc", ""))
    diagnostic_epoch = time_of_day(str(values.get("gga_utc_time", "")))
    if raw_epoch is None or diagnostic_epoch != raw_epoch:
        return "diagnostic_gga_epoch_mismatch"
    if finite(values.get("gga_quality")) != gga.get("quality"):
        return "diagnostic_gga_quality_mismatch"
    if str(values.get("fix_valid", "")).lower() != "true" or values.get("invalid_reason") not in ("", None):
        return "node_fix_invalid:" + str(values.get("invalid_reason") or "unknown")
    for key in ("data_age_s", "gga_age_s", "fix_epoch_age_s"):
        age = finite(values.get(key))
        if age is None or not 0 <= age or age + diagnostic_age_s > timeout:
            return "node_" + key + "_missing_or_stale"
    age = finite(values.get("effective_differential_age_s"))
    if age is None or not 0 <= age or age + diagnostic_age_s > max_age:
        return "differential_age_missing_or_stale"
    serial_age = finite(values.get("rtcm_serial_write_age_s", values.get("last_rtcm_write_age_s")))
    if serial_age is not None and (serial_age < 0 or serial_age + diagnostic_age_s > max_age):
        return "serial_rtcm_write_stale"
    # An old receiver may freeze age while continuing quality=4. The runtime
    # also requires recent CRC-valid RTCM written to this serial session.
    freshness = str(values.get("fresh_rtk_usable", "")).lower()
    if freshness == "false":
        return "node_fresh_rtk_unusable"
    if freshness != "true":
        return "node_fresh_rtk_unproven"
    return ""


def gaps(records, started, ended, threshold):
    times = sorted(r["receipt_monotonic_s"] for r in records)
    boundaries = [started] + times + [ended]
    spans = [{"start_offset_s": a - started, "duration_s": b - a,
              "edge": index == 0 or index == len(boundaries) - 2}
             for index, (a, b) in enumerate(zip(boundaries, boundaries[1:])) if b - a > threshold]
    span = times[-1] - times[0] if len(times) >= 2 else 0
    return {"count": len(records), "receipt_hz": (len(times) - 1) / span if span > 0 else None,
            "max_gap_s": max((b - a for a, b in zip(boundaries, boundaries[1:])), default=0.0),
            "gaps_over_threshold": spans}


def nearest(records, stamps, stamp, window, predicate=None):
    index = bisect.bisect_left(stamps, stamp)
    if predicate is None:
        candidates = records[max(0, index - 1):index + 1]
    else:
        first = bisect.bisect_left(stamps, stamp - window)
        last = bisect.bisect_right(stamps, stamp + window)
        candidates = [record for record in records[first:last] if predicate(record)]
    record = min(candidates, key=lambda r: abs(r["receipt_monotonic_s"] - stamp)) if candidates else None
    return record if record and abs(record["receipt_monotonic_s"] - stamp) <= window else None


def write_csv(path, rows, fields):
    with open(path, "x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def plot_svg(path, rows):
    usable = [row for row in rows if row["accepted"]]
    width, height, padding = 900, 660, 70
    if not usable:
        content = '<text x="40" y="80">No qualified fresh RTK trajectory points.</text>'
    else:
        xs, ys = [r["east_m"] for r in usable], [r["north_m"] for r in usable]
        xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)
        xmid, ymid = (xmin + xmax) / 2, (ymin + ymax) / 2
        scale = min((width - 2 * padding) / max(xmax - xmin, 0.5),
                    (height - 2 * padding - 60) / max(ymax - ymin, 0.5))
        def point(row):
            return (width / 2 + (row["east_m"] - xmid) * scale,
                    height / 2 - (row["north_m"] - ymid) * scale)
        # Invalid intervals break polylines; they are never visually bridged.
        segments, segment = [], []
        for row in rows:
            if not row["accepted"]:
                if segment:
                    segments.append(segment)
                segment = []
            else:
                segment.append("{:.2f},{:.2f}".format(*point(row)))
        if segment:
            segments.append(segment)
        content = ''.join('<polyline points="{}" fill="none" stroke="#1769aa" stroke-width="2"/>'.format(' '.join(s)) for s in segments)
        for row, color, label in ((usable[0], "#16803d", "first"), (usable[-1], "#b91c1c", "last")):
            x, y = point(row)
            content += '<circle cx="{:.2f}" cy="{:.2f}" r="5" fill="{}"/><text x="{:.2f}" y="{:.2f}">{}</text>'.format(x, y, color, x + 8, y - 8, label)
        content += '<text x="40" y="600">East range {:.3f}..{:.3f} m; North range {:.3f}..{:.3f} m; equal axis scale</text>'.format(xmin, xmax, ymin, ymax)
    with open(path, "x", encoding="utf-8") as stream:
        stream.write('<svg xmlns="http://www.w3.org/2000/svg" width="900" height="660" viewBox="0 0 900 660"><rect width="900" height="660" fill="white"/><g font-family="sans-serif" font-size="14" fill="#222"><text x="40" y="30">Fixed-origin WGS84 ENU antenna trajectory: East right, North up</text>' + content + '<text x="40" y="635">Relative motion only. Absolute accuracy and GNSS/FAST-LIO alignment are unverified.</text></g></svg>\n')


def analyze(args):
    directory = Path(args.input).resolve()
    with open(directory / "messages.jsonl", encoding="utf-8") as stream:
        records = [json.loads(line) for line in stream if line.strip()]
    records.sort(key=lambda r: r["receipt_monotonic_s"])
    metadata_path = directory / "capture.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8")) if metadata_path.exists() else {}
    started = metadata.get("started_monotonic_s", records[0]["receipt_monotonic_s"] if records else 0.0)
    end_path = directory / "capture_end.json"
    end = json.loads(end_path.read_text(encoding="utf-8")) if end_path.exists() else {}
    ended = end.get("ended_monotonic_s", records[-1]["receipt_monotonic_s"] if records else started)
    by_source = {source: [r for r in records if r.get("source") == source] for source in ("nmea", "fix", "diagnostics")}
    for record in by_source["nmea"]:
        record["inspection"] = inspect_nmea(record["raw"])
    ggas = [r for r in by_source["nmea"] if r["inspection"]["gga"] is not None]
    fix_stamps = [r["receipt_monotonic_s"] for r in by_source["fix"]]
    diag_stamps = [r["receipt_monotonic_s"] for r in by_source["diagnostics"]]
    track, rows = FixedOriginTrack(directory), []
    previous_epoch, previous_time = None, None
    for record in ggas:
        stamp, gga = record["receipt_monotonic_s"], record["inspection"]["gga"]
        epoch = time_of_day(gga["utc"])
        delta = None if epoch is None or previous_epoch is None else (epoch - previous_epoch) % 86400
        advancing = delta is not None and 0 < delta < 43200 and previous_time is not None and stamp - previous_time <= args.timeout
        if epoch is not None and (previous_epoch is None or (delta is not None and 0 < delta < 43200)):
            previous_epoch, previous_time = epoch, stamp
        fix = nearest(by_source["fix"], fix_stamps, stamp, args.match_window)
        diagnostic = nearest(
            by_source["diagnostics"], diag_stamps, stamp, args.diagnostic_window,
            predicate=lambda item: time_of_day(str(diagnostic_values(item).get("gga_utc_time", ""))) == epoch
            and finite(diagnostic_values(item).get("gga_quality")) == gga["quality"])
        diagnostic_age = max(0.0, stamp - diagnostic["receipt_monotonic_s"]) if diagnostic else 0.0
        reason = qualify(gga, fix, diagnostic, advancing, args.max_differential_age, args.timeout, diagnostic_age)
        result = track.add(gga["latitude"], gga["longitude"], gga["altitude_ellipsoid_m"], not reason, reason,
                           {"record": record["record"], "receipt_utc": record["receipt_utc"], "gga_utc": gga["utc"]})
        row = {"record": record["record"], "receipt_utc": record["receipt_utc"], "offset_s": stamp - started,
               "receipt_monotonic_s": stamp, "phase": record.get("phase", ""), "gga_utc": gga["utc"],
               "quality": gga["quality"], "raw_quality4": gga["quality"] == 4,
               "differential_age_s": gga["differential_age_s"], "latitude": gga["latitude"],
               "longitude": gga["longitude"], "altitude_ellipsoid_m": gga["altitude_ellipsoid_m"],
               "fix_record": fix["record"] if fix else None, "diagnostic_record": diagnostic["record"] if diagnostic else None}
        row.update(result)
        values = diagnostic_values(diagnostic)
        row["fresh_rtk_reason"] = values.get("fresh_rtk_reason", "")
        row["effective_differential_age_s"] = finite(values.get("effective_differential_age_s"))
        row["serial_rtcm_write_age_s"] = finite(values.get("rtcm_serial_write_age_s", values.get("last_rtcm_write_age_s")))
        rows.append(row)
    output = directory / ("analysis_" + datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S%fZ"))
    output.mkdir()
    fields = ["record", "receipt_utc", "offset_s", "phase", "gga_utc", "quality", "raw_quality4", "accepted", "reason",
              "fresh_rtk_reason", "differential_age_s", "effective_differential_age_s", "serial_rtcm_write_age_s",
              "latitude", "longitude", "altitude_ellipsoid_m", "east_m", "north_m", "up_m", "fix_record", "diagnostic_record"]
    write_csv(output / "trajectory.csv", rows, fields)
    plot_svg(output / "trajectory.svg", rows)
    usable = [row for row in rows if row["accepted"]]
    diagnostic_rows = []
    for record in by_source["diagnostics"]:
        values = diagnostic_values(record)
        if values:
            entry = dict(values)
            entry.update({"record": record["record"], "receipt_utc": record["receipt_utc"],
                          "offset_s": record["receipt_monotonic_s"] - started, "phase": record.get("phase", "")})
            diagnostic_rows.append(entry)
    diagnostic_fields = ["record", "receipt_utc", "offset_s", "phase"] + sorted(set(k for r in diagnostic_rows for k in r) - {"record", "receipt_utc", "offset_s", "phase"})
    write_csv(output / "diagnostic_samples.csv", diagnostic_rows, diagnostic_fields)
    markers_path = directory / "markers.jsonl"
    markers = [json.loads(line) for line in markers_path.read_text(encoding="utf-8").splitlines() if line.strip()] if markers_path.exists() else []
    marked = []
    for marker in markers:
        candidate = min(rows, key=lambda r: abs(r["receipt_monotonic_s"] - marker["monotonic_s"])) if rows else None
        in_window = candidate is not None and abs(candidate["receipt_monotonic_s"] - marker["monotonic_s"]) <= args.timeout
        matched = in_window and candidate["accepted"]
        item = {"label": marker["label"], "receipt_utc": marker["utc"], "matched": matched,
                "reason": "" if matched else (candidate["reason"] if in_window else "no_gga_point_within_timeout")}
        if matched:
            item.update({key: candidate[key] for key in ("record", "east_m", "north_m", "up_m")})
            item["time_delta_s"] = candidate["receipt_monotonic_s"] - marker["monotonic_s"]
        marked.append(item)
    pairs = []
    valid_marks = [m for m in marked if m["matched"]]
    for index, a in enumerate(valid_marks):
        for b in valid_marks[index + 1:]:
            de, dn, du = (b[k] - a[k] for k in ("east_m", "north_m", "up_m"))
            pairs.append({"from": a["label"], "to": b["label"], "east_delta_m": de, "north_delta_m": dn,
                          "up_delta_m": du, "horizontal_distance_m": math.hypot(de, dn), "distance_3d_m": math.sqrt(de * de + dn * dn + du * du)})
    write_csv(output / "marked_points.csv", marked, ["label", "receipt_utc", "matched", "reason", "record", "time_delta_s", "east_m", "north_m", "up_m"])
    write_csv(output / "point_distances.csv", pairs, ["from", "to", "east_delta_m", "north_delta_m", "up_delta_m", "horizontal_distance_m", "distance_3d_m"])
    ages = sorted(r["differential_age_s"] for r in rows if finite(r.get("differential_age_s")) is not None)
    phases = {}
    for phase in sorted(set(r["phase"] for r in rows)):
        selected = [r for r in rows if r["phase"] == phase]
        durations = [r["offset_s"] for r in selected]
        run_start, run_end, longest = None, None, 0.0
        for row in selected:
            if not row["accepted"]:
                run_start, run_end = None, None
            else:
                if run_start is None or row["offset_s"] - run_end > args.timeout:
                    run_start = row["offset_s"]
                run_end = row["offset_s"]
                longest = max(longest, run_end - run_start)
        phases[phase] = {"gga_count": len(selected), "observed_gga_span_s": max(durations) - min(durations) if durations else 0,
                         "raw_quality4_count": sum(r["raw_quality4"] for r in selected),
                         "raw_quality4_ratio": sum(r["raw_quality4"] for r in selected) / len(selected),
                         "fresh_rtk_count": sum(r["accepted"] for r in selected),
                         "fresh_rtk_ratio": sum(r["accepted"] for r in selected) / len(selected),
                         "longest_sampled_fresh_rtk_span_s": longest,
                         "rejection_reasons": dict(collections.Counter(r["reason"] for r in selected if not r["accepted"]))}
    summary = {"generated_utc": utc_now(), "elapsed_s": ended - started, "actual_ros_messages_only": True,
               "capture_end_recorded": bool(end), "capture_interrupted": end.get("interrupted"),
               "requested_duration_met": ended - started >= metadata.get("requested_seconds", float("inf")) - 0.1,
               "input_sha256": hashlib.sha256((directory / "messages.jsonl").read_bytes()).hexdigest(),
               "origin_file": str(directory / "origin.json") if track.origin else None,
               "gga_count": len(rows), "raw_quality_counts": dict(collections.Counter(str(r["quality"]) for r in rows)),
               "raw_quality4_ratio": sum(r["raw_quality4"] for r in rows) / len(rows) if rows else None,
               "fresh_rtk_ratio": len(usable) / len(rows) if rows else None,
               "raw_quality4_rejected_count": sum(r["raw_quality4"] and not r["accepted"] for r in rows),
               "first_gga_offset_s": rows[0]["offset_s"] if rows else None,
               "time_to_first_raw_quality4_s": next((r["offset_s"] for r in rows if r["raw_quality4"]), None),
               "time_to_first_fresh_rtk_s": usable[0]["offset_s"] if usable else None,
               "first_gga_to_first_raw_quality4_s": next((r["offset_s"] - rows[0]["offset_s"] for r in rows if r["raw_quality4"]), None),
               "first_gga_to_first_fresh_rtk_s": usable[0]["offset_s"] - rows[0]["offset_s"] if usable and rows else None,
               "receiver_differential_age_s": {"samples": len(ages), "min": ages[0] if ages else None,
                                                "median": ages[len(ages) // 2] if ages else None,
                                                "p95": ages[min(len(ages) - 1, int(len(ages) * 0.95))] if ages else None,
                                                "max": ages[-1] if ages else None},
               "topic_statistics": {source: gaps(items, started, ended, args.timeout) for source, items in by_source.items()},
               "gga_statistics": gaps(ggas, started, ended, args.timeout),
               "vehicle_topic_counts": dict(collections.Counter(r["topic"] for r in records if r.get("source") == "vehicle")),
               "bad_nmea_count": sum(not r["inspection"]["checksum_valid"] for r in by_source["nmea"]),
               "phases": phases, "rejection_reasons": dict(collections.Counter(r["reason"] for r in rows if not r["accepted"])),
               "freshness_policy": {"raw_quality": 4, "max_differential_age_s": args.max_differential_age,
                                    "timeout_s": args.timeout, "fix_match_window_s": args.match_window,
                                    "diagnostic_window_s": args.diagnostic_window,
                                    "requires": "advancing checksum-valid GGA, matching valid ROS fix, finite ellipsoid altitude, fresh ages, explicit fresh_rtk_usable=true"},
               "limits": ["Phase labels and markers are operator annotations, not proof of physical actions.",
                          "Fixed quality and ENU/fitted residuals do not prove centimetre absolute accuracy.",
                          "FAST-LIO comparison requires reviewed clock alignment, coordinate transform, and antenna lever arm.",
                          "Dropped ROS messages and physical movement cannot be reconstructed from missing evidence."]}
    json_write(output / "summary.json", summary)
    print(json.dumps({"analysis_directory": str(output), "summary": summary}, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


def mark(args):
    directory = Path(args.input).resolve()
    if not (directory / "capture.json").is_file():
        raise ValueError("--input must be an actual capture run directory")
    event = {"label": args.label, "note": args.note, "utc": utc_now(), "monotonic_s": time.monotonic(), "hostname": socket.gethostname()}
    metadata = json.loads((directory / "capture.json").read_text(encoding="utf-8"))
    if metadata.get("hostname") != event["hostname"]:
        raise ValueError("mark must run on the capture host; cross-host monotonic clocks cannot be compared")
    with open(directory / "markers.jsonl", "a", encoding="utf-8") as stream:
        stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n")
    if args.phase:
        temporary = directory / ("phase_" + str(os.getpid()) + ".tmp")
        temporary.write_text(args.phase + "\n", encoding="utf-8")
        os.replace(str(temporary), str(directory / "phase.txt"))
    print(json.dumps(event, ensure_ascii=False))
    return 0


def capture(args):
    import rospy
    from diagnostic_msgs.msg import DiagnosticArray
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String
    directory = Path(args.output).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    metadata = {"version": 1, "started_utc": utc_now(), "started_monotonic_s": started,
                "hostname": socket.gethostname(), "platform": platform.platform(), "architecture": platform.machine(),
                "python": platform.python_version(), "requested_seconds": args.seconds,
                "phase_is_operator_label_only": True, "topics": list(GNSS_TOPICS) + (list(VEHICLE_TOPICS) if args.vehicle_topics else [])}
    if args.source_manifest:
        content = Path(args.source_manifest).read_bytes()
        (directory / "source_manifest.json").write_bytes(content)
        metadata["source_manifest_sha256"] = hashlib.sha256(content).hexdigest()
    rospy.init_node("rtk_evidence", anonymous=True, disable_signals=True)
    metadata["use_sim_time"] = rospy.get_param("/use_sim_time", False)
    json_write(directory / "capture.json", metadata)
    (directory / "phase.txt").write_text(args.phase + "\n", encoding="utf-8")
    lock, accepting, counts = threading.Lock(), [True], collections.Counter()
    stream = open(directory / "messages.jsonl", "x", encoding="utf-8")
    subscriptions = []
    phase = [args.phase]

    def callback(message, topic):
        nonfinite = {}
        raw_ros = ros_dict(message, nonfinite)
        entry = {"topic": topic, "source": GNSS_TOPICS.get(topic, "vehicle"), "phase": phase[0],
                 "receipt_monotonic_s": time.monotonic(), "receipt_utc": utc_now(),
                 "receipt_ros_s": rospy.Time.now().to_sec(), "ros_message": raw_ros, "nonfinite_fields": nonfinite}
        if topic == "/gnss/nmea":
            entry.update({"raw": message.data})
        elif topic == "/gnss/fix":
            entry.update({"status": message.status.status, "latitude": finite(message.latitude),
                          "longitude": finite(message.longitude), "altitude": finite(message.altitude)})
        elif topic == "/gnss/diagnostics":
            entry["statuses"] = [{"name": s.name, "level": s.level, "message": s.message,
                                  "values": {v.key: v.value for v in s.values}} for s in message.status]
        with lock:
            if not accepting[0]:
                return
            entry["record"] = sum(counts.values())
            stream.write(json.dumps(entry, ensure_ascii=False, allow_nan=False) + "\n")
            stream.flush()
            counts[topic] += 1

    for topic, message_type in (("/gnss/nmea", String), ("/gnss/fix", NavSatFix), ("/gnss/diagnostics", DiagnosticArray)):
        subscriptions.append(rospy.Subscriber(topic, message_type, callback, callback_args=topic, queue_size=2000))
    if args.vehicle_topics:
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage
        for topic in VEHICLE_TOPICS:
            message_type = TFMessage if topic in ("/tf", "/tf_static") else Odometry
            subscriptions.append(rospy.Subscriber(topic, message_type, callback, callback_args=topic, queue_size=2000))
    interrupted, next_status = False, started
    try:
        while time.monotonic() - started < args.seconds and not rospy.is_shutdown():
            try:
                phase[0] = (directory / "phase.txt").read_text(encoding="utf-8").strip()[:128]
            except OSError:
                pass
            if time.monotonic() >= next_status:
                with lock:
                    status = dict(counts)
                print(json.dumps({"elapsed_s": round(time.monotonic() - started, 1), "phase": phase[0], "topic_counts": status}, ensure_ascii=False), flush=True)
                next_status = time.monotonic() + 10
            time.sleep(0.1)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        for subscription in subscriptions:
            subscription.unregister()
        with lock:
            accepting[0] = False
            stream.close()
        json_write(directory / "capture_end.json", {"ended_utc": utc_now(), "ended_monotonic_s": time.monotonic(),
                                                  "interrupted": interrupted, "topic_counts": dict(counts)})
        rospy.signal_shutdown("RTK evidence capture finished")
    args.input = str(directory)
    return analyze(args)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    capture_parser = commands.add_parser("capture", help="Subscribe to real ROS messages on the runtime host")
    capture_parser.add_argument("--output", required=True, help="New run directory; previous raw evidence is never overwritten")
    capture_parser.add_argument("--seconds", type=float, default=360.0)
    capture_parser.add_argument("--phase", default="baseline")
    capture_parser.add_argument("--vehicle-topics", action="store_true", help="Also record /Odometry /odom /tf /tf_static if published")
    capture_parser.add_argument("--source-manifest", help="Existing exact source-content manifest to include in this run")
    analyze_parser = commands.add_parser("analyze", help="Create new derived CSV/SVG/summary from preserved messages.jsonl")
    analyze_parser.add_argument("--input", required=True)
    for subparser in (capture_parser, analyze_parser):
        subparser.add_argument("--match-window", type=float, default=0.5)
        subparser.add_argument("--diagnostic-window", type=float, default=1.5)
        subparser.add_argument("--max-differential-age", type=float, default=5.0)
        subparser.add_argument("--timeout", type=float, default=2.5)
    mark_parser = commands.add_parser("mark", help="Record an operator point/event on the capture host")
    mark_parser.add_argument("--input", required=True)
    mark_parser.add_argument("--label", required=True)
    mark_parser.add_argument("--note", default="")
    mark_parser.add_argument("--phase", help="Also change phase for following messages, e.g. rtcm_off")
    args = parser.parse_args()
    for name in ("seconds", "match_window", "diagnostic_window", "max_differential_age", "timeout"):
        if hasattr(args, name) and (finite(getattr(args, name)) is None or getattr(args, name) <= 0):
            parser.error("--" + name.replace("_", "-") + " must be positive and finite")
    if hasattr(args, "seconds") and args.seconds > 86400:
        parser.error("--seconds must be <=86400")
    return {"capture": capture, "analyze": analyze, "mark": mark}[args.command](args)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError) as error:
        print("RTK evidence error: " + str(error), file=sys.stderr)
        raise SystemExit(2)
