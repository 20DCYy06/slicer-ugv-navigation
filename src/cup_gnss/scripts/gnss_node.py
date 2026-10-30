#!/usr/bin/env python3
"""ROS1 live GNSS bridge. Uses wall-loop diagnostics even with paused ROS time."""
import math
import json
import os
from pathlib import Path
import queue
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from cup_gnss.core import GnssModel
from cup_gnss.sources import SerialSource, TcpSource
from cup_gnss.upstream_ntrip import UpstreamNtripClient as NtripClient
from cup_gnss.upstream_nmea import NmeaFixAdapter
from cup_gnss.quality import apply_rtk_policy


def text(value):
    if value is None:
        return "unknown"
    if isinstance(value, float):
        return "{:.3f}".format(value) if math.isfinite(value) else "unknown"
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def positive_param(name, default):
    value = float(rospy.get_param(name, default))
    if not math.isfinite(value) or value <= 0:
        raise ValueError(name + " must be positive and finite")
    return value


def main():
    rospy.init_node("cup_gnss")
    source_name = str(rospy.get_param("~source", "tcp"))
    model = GnssModel(
        source=source_name,
        data_timeout_s=positive_param("~data_timeout_s", 2.5),
        max_differential_age_s=positive_param("~max_differential_age_s", 5.0),
        max_source_age_s=positive_param("~max_source_age_s", 2.0),
        future_tolerance_s=float(rospy.get_param("~future_tolerance_s", 0.5)),
    )
    fix_adapter = NmeaFixAdapter()
    ntrip_enabled = bool(rospy.get_param("~ntrip_enabled", False))
    pause_file = str(rospy.get_param("~rtcm_pause_file", "/home/cup/.cup_gnss/rtcm.paused"))
    if ntrip_enabled and source_name != "serial":
        raise ValueError("NTRIP requires the single-owner serial source")
    events = queue.Queue(maxsize=256)
    if source_name == "tcp":
        port = int(rospy.get_param("~tcp_port", 10110))
        if not 1 <= port <= 65535:
            raise ValueError("tcp_port outside 1..65535")
        worker = TcpSource(events, host=str(rospy.get_param("~tcp_host", "host.docker.internal")), port=port)
    elif source_name == "serial":
        port = str(rospy.get_param("~serial_port", "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5185028488-if00"))
        baud = int(rospy.get_param("~baud", 115200))
        if baud <= 0:
            raise ValueError("baud must be positive")
        worker = SerialSource(events, port=port, baud=baud, pause_file=pause_file,
                              expected_serial=str(rospy.get_param("~expected_usb_serial", "")))
        rospy.loginfo("Serial owner: 8N1/no flow control, DTR=RTS=false; RTCM only when explicitly enabled")
    else:
        raise ValueError("source must be tcp or serial; replay sources are not supported")
    ntrip = None
    evidence_files = []
    frame_callback = None
    evidence_dir = str(rospy.get_param("~rtcm_evidence_dir", ""))
    if ntrip_enabled and evidence_dir:
        directory = Path(evidence_dir)
        directory.mkdir(mode=0o700, parents=True, exist_ok=False)
        rx_stream = (directory / "received.rtcm3").open("xb", buffering=0)
        tx_stream = (directory / "serial-written.rtcm3").open("xb", buffering=0)
        frame_log = (directory / "frames.jsonl").open("x", buffering=1)
        evidence_files = [rx_stream, tx_stream, frame_log]

        def record_frame(event):
            frame = event["frame"]
            record = {key: value for key, value in event.items() if key != "frame"}
            record.update(wall_time_s=time.time(), length=len(frame))
            if event["direction"] == "rx":
                rx_stream.write(frame)
            elif event["accepted_count"]:
                tx_stream.write(frame[:event["accepted_count"]])
            frame_log.write(json.dumps(record, sort_keys=True) + "\n")
        frame_callback = record_frame
    if ntrip_enabled:
        ntrip = NtripClient(
            credentials_file=str(rospy.get_param("~ntrip_credentials_file", "/home/cup/.cup_gnss/ntrip_credentials.json")),
            gga_provider=worker.get_gga, rtcm_writer=worker.write_rtcm,
            host=str(rospy.get_param("~ntrip_host", "rtk.ntrip.qxwz.com")),
            port=int(rospy.get_param("~ntrip_port", 8002)),
            mountpoint=str(rospy.get_param("~ntrip_mountpoint", "AUTO")),
            pause_file=pause_file, frame_callback=frame_callback)

    def combined_snapshot(now=None):
        return apply_rtk_policy(
            model.snapshot(now), ntrip.snapshot() if ntrip else None,
            worker.rtcm_write_snapshot() if source_name == "serial" else None,
            max_age=model.max_differential_age_s)

    frame_id = str(rospy.get_param("~frame_id", "gnss_link"))
    nmea_pub = rospy.Publisher("/gnss/nmea", String, queue_size=100)
    fix_pub = rospy.Publisher("/gnss/fix", NavSatFix, queue_size=10)
    diag_pub = rospy.Publisher("/gnss/diagnostics", DiagnosticArray, queue_size=10)

    def publish_fix(snapshot):
        raw = model.latest_gga_raw.get("raw") if model.latest_gga_raw else None
        try:
            fix = fix_adapter.to_fix(raw, snapshot, rospy.Time.now(), frame_id)
        except ValueError:
            # A library/policy mismatch must never keep a valid cached fix.
            # The next real advancing GGA may recover; no raw library text leaks.
            model._reject("upstream_message_adaptation_failed")
            fix = fix_adapter.to_fix(None, combined_snapshot(), rospy.Time.now(), frame_id)
        fix_pub.publish(fix)

    worker.start()
    if ntrip:
        ntrip.start()
    next_diagnostics = time.monotonic()
    try:
        while not rospy.is_shutdown():
            try:
                event = events.get(timeout=0.05)
            except queue.Empty:
                event = None
            if event is not None:
                result = model.ingest(event)
                if result["raw"] is not None:
                    nmea_pub.publish(String(data=result["raw"]))
                snapshot = combined_snapshot()
                if result["publish_fix"] or (result["changed"] and not snapshot["fix"]["valid"]):
                    publish_fix(snapshot)
            now = time.monotonic()
            if now >= next_diagnostics:
                snapshot = combined_snapshot(now)
                status = DiagnosticStatus()
                status.name = "cup_gnss/receiver"
                status.hardware_id = source_name
                status.level = snapshot["level"]
                status.message = snapshot["message"]
                status.values = [KeyValue(key=key, value=text(value))
                                 for key, value in snapshot["diagnostics"].items()]
                message = DiagnosticArray()
                message.header.stamp = rospy.Time.now()
                message.status = [status]
                diag_pub.publish(message)
                # No valid-position republishing from a cache. Invalid heartbeat
                # prevents a previous valid fix appearing current after disconnect.
                if not snapshot["fix"]["valid"]:
                    publish_fix(snapshot)
                next_diagnostics = now + 1.0
    finally:
        if ntrip:
            ntrip.stop()
            ntrip.join(timeout=4.0)
        worker.stop()
        worker.join(timeout=3.0)
        for stream in evidence_files:
            stream.close()
        if worker.is_alive():
            rospy.logwarn("GNSS source thread did not stop before the 3-second shutdown deadline")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, TypeError) as exc:
        rospy.logfatal(str(exc))
        raise SystemExit(2)
