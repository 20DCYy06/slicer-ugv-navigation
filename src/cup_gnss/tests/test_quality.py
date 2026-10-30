"""Policy fixtures use real NMEA ingestion; not receiver acceptance evidence."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from cup_gnss.core import GnssModel
from cup_gnss.quality import apply_rtk_policy


def nmea(body):
    checksum = 0
    for byte in body.encode("ascii"):
        checksum ^= byte
    return "$" + body + "*%02X" % checksum


def model_snapshot(quality=4, differential_age="1.0", session="receiver-current"):
    model = GnssModel(source="serial")
    model.ingest({"kind": "transport", "connected": True}, now_wall=100, now_mono=10)
    model.ingest({"v": 1, "kind": "status", "session": session,
                  "connected": True, "sent_at": 100}, now_wall=100, now_mono=10)
    for sequence in (1, 2):
        raw = nmea("GNGGA,12000%d.000,3112.123456,N,12130.654321,E,%d,20,0.7,15.0,M,0.0,M,%s,0001"
                   % (sequence, quality, differential_age))
        model.ingest({"v": 1, "kind": "nmea", "session": session, "seq": sequence,
                      "captured_at": 99 + sequence, "nmea": raw},
                     now_wall=99 + sequence, now_mono=9 + sequence)
    return model.snapshot(11)


def writes(age=0.1, session="receiver-current"):
    return {"rtcm_serial_write_age_s": age, "rtcm_serial_write_session": session}


class RtkPolicyCase(unittest.TestCase):
    def apply(self, snapshot=None, ntrip=None, serial=None):
        return apply_rtk_policy(model_snapshot() if snapshot is None else snapshot,
                                {"enabled": True, "fatal": False} if ntrip is None else ntrip,
                                writes() if serial is None else serial, max_age=5.0)

    def assert_fresh(self, result):
        self.assertTrue(result["fix"]["valid"])
        self.assertTrue(result["diagnostics"]["fresh_rtk_usable"])
        self.assertEqual(result["diagnostics"]["fresh_rtk_reason"], "")
        self.assertEqual(result["diagnostics"]["gga_quality"], 4)

    def assert_invalid(self, result, reason):
        self.assertFalse(result["fix"]["valid"])
        self.assertEqual(result["fix"]["status"], -1)
        self.assertTrue(math.isnan(result["fix"]["latitude"]))
        self.assertFalse(result["diagnostics"]["fresh_rtk_usable"])
        self.assertEqual(result["diagnostics"]["fresh_rtk_reason"], reason)
        self.assertEqual(result["diagnostics"]["invalid_reason"], reason)

    def test_fixed_requires_fresh_receiver_age_and_same_session_successful_write(self):
        self.assert_fresh(self.apply())
        self.assert_fresh(self.apply(serial=writes(age=5.0)))
        self.assertEqual(self.apply()["fix"]["position_covariance_type"], 0)

    def test_raw_fixed_but_receiver_reports_stale_or_unknown_age(self):
        for age in ("6.0", ""):
            with self.subTest(receiver_age=age):
                result = self.apply(snapshot=model_snapshot(differential_age=age))
                self.assert_invalid(result, "differential_age_missing_or_stale")
                self.assertEqual(result["diagnostics"]["gga_quality"], 4)

    def test_frozen_receiver_fixed_and_age_cannot_hide_stopped_corrections(self):
        # The receiver keeps emitting current advancing quality=4 / age=1.0 GGA.
        # Independent successful UART-write age still ends its usable lifetime.
        result = self.apply(serial=writes(age=30.0))
        self.assert_invalid(result, "rtcm_serial_write_missing_or_stale")
        self.assertEqual(result["diagnostics"]["gga_quality"], 4)
        self.assertEqual(result["diagnostics"]["differential_age_s"], 1.0)

    def test_pause_preserves_short_freshness_grace_and_expires_it(self):
        ntrip = {"enabled": True, "fatal": False, "paused": True, "connected": False}
        self.assert_fresh(self.apply(ntrip=ntrip, serial=writes(age=1.0)))
        self.assert_invalid(self.apply(ntrip=ntrip, serial=writes(age=5.001)),
                            "rtcm_serial_write_missing_or_stale")

    def test_new_serial_session_cannot_use_earlier_session_writes(self):
        result = self.apply(snapshot=model_snapshot(session="receiver-new"),
                            serial=writes(age=0.01, session="receiver-previous"))
        self.assert_invalid(result, "rtcm_serial_write_missing_or_stale")
        self.assert_fresh(self.apply(snapshot=model_snapshot(session="receiver-new"),
                                     serial=writes(age=0.01, session="receiver-new")))

    def test_disconnected_source_cleared_write_state_cannot_claim_freshness(self):
        self.assert_invalid(self.apply(serial=writes(age=None, session="")),
                            "rtcm_serial_write_missing_or_stale")

    def test_invalid_write_age_values_are_rejected(self):
        for age in (None, -0.1, math.nan, math.inf, True, "0.01"):
            with self.subTest(age=age):
                self.assert_invalid(self.apply(serial=writes(age=age)),
                                    "rtcm_serial_write_missing_or_stale")

    def test_fatal_serial_error_invalidates_even_recent_fixed_solution(self):
        result = self.apply(ntrip={"enabled": True, "fatal": True})
        self.assert_invalid(result, "rtcm_forwarding_fatal")

    def test_ordinary_single_fix_remains_valid_without_live_corrections(self):
        for enabled in (False, True):
            with self.subTest(ntrip_enabled=enabled):
                result = self.apply(snapshot=model_snapshot(quality=1, differential_age=""),
                                    ntrip={"enabled": enabled, "fatal": False}, serial=writes(age=None))
                self.assertTrue(result["fix"]["valid"])
                self.assertEqual(result["fix"]["status"], 0)
                self.assertFalse(result["diagnostics"]["fresh_rtk_usable"])
                self.assertEqual(result["diagnostics"]["fresh_rtk_reason"], "receiver_not_rtk_fixed")

    def test_float_or_differential_is_useful_only_with_recent_enabled_chain(self):
        for quality in (2, 5):
            with self.subTest(quality=quality):
                result = self.apply(snapshot=model_snapshot(quality=quality))
                self.assertTrue(result["fix"]["valid"])
                self.assertFalse(result["diagnostics"]["fresh_rtk_usable"])
                self.assertEqual(result["diagnostics"]["fresh_rtk_reason"], "receiver_not_rtk_fixed")
                stale = self.apply(snapshot=model_snapshot(quality=quality), serial=writes(age=6.0))
                self.assertFalse(stale["fix"]["valid"])
                self.assertEqual(stale["diagnostics"]["invalid_reason"], "rtcm_serial_write_missing_or_stale")

    def test_external_fixed_flag_is_not_advertised_as_verified_live_rtk(self):
        result = self.apply(ntrip={"enabled": False}, serial=writes(age=None))
        self.assertTrue(result["fix"]["valid"])
        self.assertFalse(result["diagnostics"]["fresh_rtk_usable"])
        self.assertEqual(result["diagnostics"]["gga_quality"], 4)
        self.assertEqual(result["diagnostics"]["fresh_rtk_reason"], "live_corrections_not_enabled")


if __name__ == "__main__":
    unittest.main()
