#!/usr/bin/env python3
"""Read-only terminal monitor of the live GNSS diagnostic topic."""
import argparse
import json
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--once", action="store_true", help="Print one fresh diagnostic, then exit")
    parser.add_argument("--json", action="store_true", help="Print diagnostic values as JSON")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("cup_gnss_status", anonymous=True, disable_signals=False)
    done = threading.Event()
    last = [None]

    def display(message):
        for status in message.status:
            if status.name != "cup_gnss/receiver":
                continue
            values = {pair.key: pair.value for pair in status.values}
            values.update({"level": status.level, "message": status.message})
            last[0] = time.monotonic()
            if args.json:
                print(json.dumps(values, ensure_ascii=False), flush=True)
            else:
                print("{connection} | NMEA {hz} Hz, GGA {gga_hz} Hz | q={q} {fix_type} | "
                      "sat={sat} HDOP={hdop} diff={diff}s (ok={diff_ok}) | "
                      "age={age}s GGA age={gga_age}s | fix_valid={valid} | "
                      "fresh_RTK={rtk} ({reason}) RTCM={rtcm} write_age={write_age}s | {message}".format(
                          connection="serial=" + values.get("serial_connected", "unknown") +
                          ",transport=" + values.get("transport_connected", "unknown"),
                          hz=values.get("nmea_hz", "?"), gga_hz=values.get("gga_hz", "?"),
                          q=values.get("gga_quality", "?"), fix_type=values.get("fix_type", "?"),
                          sat=values.get("satellites", "?"), hdop=values.get("hdop", "?"),
                          diff=values.get("differential_age_s", "?"), diff_ok=values.get("differential_age_ok", "?"),
                          age=values.get("data_age_s", "?"), gga_age=values.get("gga_age_s", "?"),
                          valid=values.get("fix_valid", "?"), message=status.message,
                          rtk=values.get("fresh_rtk_usable", "?"), reason=values.get("fresh_rtk_reason", "?"),
                          rtcm=values.get("ntrip_state", "disabled"), write_age=values.get("rtcm_serial_write_age_s", "?")), flush=True)
            if args.once:
                done.set()

    subscriber = rospy.Subscriber("/gnss/diagnostics", DiagnosticArray, display, queue_size=1)
    started, last_warning = time.monotonic(), 0.0
    while not rospy.is_shutdown() and not done.wait(0.1):
        now = time.monotonic()
        if now - (last[0] if last[0] is not None else started) > 3.0 and now - last_warning > 3.0:
            print("GNSS diagnostics timeout: no new /gnss/diagnostics for 3 seconds", flush=True)
            last_warning = now
            if args.once:
                return 3
    subscriber.unregister()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
