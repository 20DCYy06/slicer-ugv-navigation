"""Offline unit fixtures only. These tests are not live-device acceptance."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from cup_gnss.core import GnssModel, coordinate, parse_nmea
from cup_gnss.sources import LineFramer


def nmea(body):
    check = 0
    for byte in body.encode("ascii"):
        check ^= byte
    return "$" + body + "*{:02X}".format(check)


def gga(epoch="120000.00", q=1, diff="", lat="3156.28660023", ns="N",
        lon="11847.09918862", ew="E", talker="GN", altitude="17.7102", geoid="2.2110"):
    return nmea("{},{} ,{},{},{},{},{},30,0.6,{},M,{},M,{},3854".format(
        talker + "GGA", epoch, lat, ns, lon, ew, q, altitude, geoid, diff).replace(" ,", ","))


class ModelCase(unittest.TestCase):
    def setUp(self):
        self.model = GnssModel()
        self.wall, self.mono, self.seq, self.session = 1000.0, 10.0, 0, "serial-session-a"
        self.model.ingest({"kind": "transport", "connected": True}, self.wall, self.mono)
        self.status(True)

    def status(self, connected, session=None, sent_at=None):
        return self.model.ingest({"v": 1, "kind": "status", "session": session or self.session,
                                  "connected": connected, "port": "TEST_OFFLINE",
                                  "last_rx_at": None,
                                  "sent_at": self.wall if sent_at is None else sent_at}, self.wall, self.mono)

    def feed(self, raw, step=0.0, captured_at=None, session=None, seq=None, receipt_mono=None):
        self.wall += step
        self.mono += step
        self.seq += 1
        return self.model.ingest({"v": 1, "kind": "nmea", "session": session or self.session,
                                  "seq": self.seq if seq is None else seq,
                                  "captured_at": self.wall if captured_at is None else captured_at,
                                  "nmea": raw,
                                  "_received_wall": self.wall,
                                  "_received_mono": self.mono if receipt_mono is None else receipt_mono},
                                 self.wall, self.mono)

    def fixed(self, q=4, diff="1.0", **kwargs):
        self.feed(gga("120000.00", q=q, diff=diff, **kwargs))
        self.feed(gga("120001.00", q=q, diff=diff, **kwargs), step=1)
        return self.model.snapshot(self.mono)

    def test_checksum_and_strict_coordinates(self):
        raw = gga()
        self.assertIsNotNone(parse_nmea(raw))
        self.assertIsNone(parse_nmea(raw[:-2] + "00"))
        self.assertIsNone(coordinate("3199.0", "N", True))
        self.assertIsNone(coordinate("9000.01", "N", True))
        self.assertIsNone(coordinate("18000.01", "E", False))
        self.assertIsNone(coordinate("3156.2", "", True))
        self.assertIsNone(parse_nmea(raw + "\r\n"))

    def test_real_zero_coordinates_are_valid_with_valid_quality(self):
        state = self.fixed(q=1, diff="", lat="0000.000", lon="00000.000")
        self.assertTrue(state["fix"]["valid"])
        self.assertEqual(state["fix"]["latitude"], 0)
        self.assertEqual(state["fix"]["longitude"], 0)

    def test_two_epochs_and_covariance_unknown(self):
        first = self.feed(gga())
        self.assertTrue(first["publish_fix"])
        self.assertEqual(self.model.snapshot(self.mono)["diagnostics"]["invalid_reason"], "awaiting_advancing_gga")
        self.feed(gga("120001.00"), step=1)
        state = self.model.snapshot(self.mono)
        self.assertTrue(state["fix"]["valid"])
        self.assertEqual(state["fix"]["status"], 0)
        self.assertEqual(state["fix"]["position_covariance_type"], 0)
        self.assertEqual(state["fix"]["position_covariance"], [0.0] * 9)
        self.assertAlmostEqual(state["fix"]["altitude"], 19.9212)
        self.assertEqual(state["fix"]["service"], 0)

    def test_fixed_float_difference_is_retained(self):
        state = self.fixed()
        self.assertTrue(state["fix"]["valid"])
        self.assertEqual(state["fix"]["status"], 2)
        self.assertEqual(state["diagnostics"]["gga_quality"], 4)
        self.assertEqual(state["diagnostics"]["fix_type"], "RTK fixed")
        self.feed(gga("120002.00", q=5, diff="1"), step=1)
        state = self.model.snapshot(self.mono)
        self.assertTrue(state["fix"]["valid"])
        self.assertEqual(state["fix"]["status"], 2)
        self.assertEqual(state["diagnostics"]["fix_type"], "RTK float")
        self.assertEqual(state["level"], 1)

    def test_missing_stale_negative_corrections_fail_closed(self):
        for q in (2, 4, 5):
            for diff in ("", "6", "-1", "nan"):
                with self.subTest(q=q, diff=diff):
                    state = self.fixed(q=q, diff=diff)
                    self.assertFalse(state["fix"]["valid"])
                    self.assertTrue(math.isnan(state["fix"]["latitude"]))
                    self.assertEqual(state["diagnostics"]["gga_quality"], q)
                    self.setUp()

    def test_correction_age_advances_without_new_gga(self):
        self.fixed(diff="4.5")
        self.assertFalse(self.model.snapshot(self.mono + 0.6)["fix"]["valid"])
        self.assertEqual(self.model.snapshot(self.mono + 0.6)["message"], "differential_age_missing_or_stale")

    def test_no_fix_estimated_manual_simulation_coordinates_nan(self):
        for q in (0, 6, 7, 8, 9):
            with self.subTest(q=q):
                state = self.fixed(q=q, diff="1")
                self.assertFalse(state["fix"]["valid"])
                self.assertEqual(state["fix"]["status"], -1)
                for key in ("latitude", "longitude", "altitude"):
                    self.assertTrue(math.isnan(state["fix"][key]))
                self.setUp()

    def test_real_indoor_blank_utc_quality_zero_is_fresh_no_fix(self):
        # Exact checksum-valid sentence observed from the live indoor receiver.
        raw = "$GNGGA,,,,,,0,,,,,,,,*78"
        result = self.feed(raw)
        self.assertEqual(result["raw"], raw)
        self.assertTrue(result["publish_fix"])
        state = self.model.snapshot(self.mono)
        self.assertEqual(state["message"], "receiver_has_no_measured_fix")
        self.assertEqual(state["level"], 1)
        self.assertEqual(state["diagnostics"]["gga_quality"], 0)
        self.assertEqual(state["diagnostics"]["gga_age_s"], 0)
        self.assertTrue(math.isnan(state["fix"]["latitude"]))
        for _ in range(5):
            self.feed(raw, step=1)
            state = self.model.snapshot(self.mono)
            self.assertEqual(state["message"], "receiver_has_no_measured_fix")
        self.assertEqual(state["diagnostics"]["gga_hz"], 1)
        self.assertEqual(self.model.snapshot(self.mono + 3)["message"], "nmea_timeout")

    def test_finite_coordinates_required_not_just_quality(self):
        state = self.fixed(q=1, lat="", ns="", lon="", ew="")
        self.assertFalse(state["fix"]["valid"])

    def test_altitude_unknown_without_geoid(self):
        state = self.fixed(q=1, geoid="")
        self.assertTrue(state["fix"]["valid"])
        self.assertTrue(math.isnan(state["fix"]["altitude"]))

    def test_known_talker_service(self):
        for talker, service in (("GP", 1), ("GL", 2), ("GA", 8), ("GB", 4), ("BD", 4), ("GD", 0), ("GN", 0)):
            state = self.fixed(q=1, talker=talker)
            self.assertEqual(state["fix"]["service"], service)
            self.setUp()

    def test_raw_invalid_checksum_is_preserved_but_not_position(self):
        bad = "$GNGGA,wrong*00"
        result = self.feed(bad)
        self.assertEqual(result["raw"], bad)
        self.assertFalse(result["publish_fix"])
        self.assertEqual(self.model.counts["checksum_errors"], 1)

    def test_stale_source_payload_cannot_become_fresh(self):
        self.fixed()
        result = self.feed(gga("120002.00", q=4, diff="1"), captured_at=self.wall - 10)
        self.assertIsNone(result["raw"])
        state = self.model.snapshot(self.mono)
        self.assertFalse(state["fix"]["valid"])
        self.assertEqual(state["message"], "source_record_too_old")

    def test_future_or_nonfinite_source_timestamp_rejected(self):
        for stamp in (self.wall + 1, math.nan, math.inf):
            self.feed(gga(), captured_at=stamp)
            self.assertFalse(self.model.snapshot(self.mono)["fix"]["valid"])
        self.assertEqual(self.model.counts["rejected_source_records"], 3)

    def test_queue_receipt_age_is_checked(self):
        self.feed(gga(), receipt_mono=self.mono - 3)
        self.assertEqual(self.model.snapshot(self.mono)["message"], "source_record_too_old")

    def test_duplicate_epoch_invalidates_even_if_new_quality_is_zero(self):
        self.fixed()
        result = self.feed(gga("120001.00", q=0, lat="", ns="", lon="", ew=""), step=0.1)
        self.assertTrue(result["publish_fix"])
        state = self.model.snapshot(self.mono)
        self.assertFalse(state["fix"]["valid"])
        self.assertEqual(state["diagnostics"]["gga_quality"], 0)
        self.assertEqual(self.model.counts["duplicate_epochs"], 1)

    def test_older_epoch_with_fixed_quality_and_stale_age_invalidates(self):
        self.fixed()
        result = self.feed(gga("115959.00", q=4, diff="500"), step=0.1)
        self.assertTrue(result["publish_fix"])
        self.assertFalse(self.model.snapshot(self.mono)["fix"]["valid"])
        self.assertEqual(self.model.counts["older_epochs"], 1)

    def test_midnight_rollover_advances(self):
        self.feed(gga("235959.00"))
        self.feed(gga("000000.00"), step=1)
        self.assertTrue(self.model.snapshot(self.mono)["fix"]["valid"])

    def test_other_nmea_and_status_do_not_refresh_gga(self):
        self.fixed()
        self.wall += 3
        self.mono += 3
        self.status(True)
        self.feed(nmea("GNRMC,120004.00,A,3156.28,N,11847.09,E,0,0,151026,,,R,S"))
        state = self.model.snapshot(self.mono)
        self.assertFalse(state["fix"]["valid"])
        self.assertEqual(state["message"], "gga_timeout")
        self.assertEqual(state["diagnostics"]["rmc_mode"], "R")
        self.assertEqual(state["diagnostics"]["rmc_navigation_status"], "S")

    def test_heartbeat_does_not_reset_existing_valid_gga(self):
        self.fixed()
        before = self.model.latest_gga
        self.status(True)
        self.assertIs(self.model.latest_gga, before)
        self.assertTrue(self.model.snapshot(self.mono)["fix"]["valid"])

    def test_disconnect_clears_coordinates_then_same_session_recovers(self):
        self.fixed()
        self.status(False)
        state = self.model.snapshot(self.mono)
        self.assertFalse(state["fix"]["valid"])
        self.assertTrue(math.isnan(state["fix"]["latitude"]))
        self.status(True)
        self.feed(gga("120010.00"), step=1)
        self.assertFalse(self.model.snapshot(self.mono)["fix"]["valid"])
        self.feed(gga("120011.00"), step=1)
        self.assertTrue(self.model.snapshot(self.mono)["fix"]["valid"])

    def test_new_serial_session_resets_sequence_and_epoch(self):
        self.fixed()
        self.session = "serial-session-b"
        self.status(True)
        self.feed(gga("120000.00"), seq=1)
        self.assertFalse(self.model.snapshot(self.mono)["fix"]["valid"])
        self.feed(gga("120001.00"), step=1, seq=2)
        self.assertTrue(self.model.snapshot(self.mono)["fix"]["valid"])

    def test_tcp_reconnect_same_serial_session_requires_new_advancement(self):
        self.fixed()
        self.model.ingest({"kind": "transport", "connected": False}, self.wall, self.mono)
        self.model.ingest({"kind": "transport", "connected": True}, self.wall, self.mono)
        self.feed(gga("120010.00"), step=1)
        self.assertFalse(self.model.snapshot(self.mono)["fix"]["valid"])
        self.feed(gga("120011.00"), step=1)
        self.assertTrue(self.model.snapshot(self.mono)["fix"]["valid"])

    def test_repeated_source_sequence_invalidates(self):
        self.fixed()
        self.feed(gga("120002.00", q=4, diff="1"), seq=self.seq)
        state = self.model.snapshot(self.mono)
        self.assertFalse(state["fix"]["valid"])
        self.assertEqual(state["message"], "duplicate_or_reordered_source_sequence")

    def test_invalid_status_timestamp_does_not_claim_new_connection(self):
        self.status(False)
        self.status(True, sent_at=math.nan)
        self.assertFalse(self.model.serial_connected)

    def test_silence_expires_and_rates_are_bounded(self):
        self.fixed()
        state = self.model.snapshot(self.mono + 3)
        self.assertFalse(state["fix"]["valid"])
        self.assertEqual(state["message"], "nmea_timeout")
        self.assertTrue(math.isnan(state["fix"]["longitude"]))
        self.assertEqual(self.model.snapshot(self.mono + 11)["diagnostics"]["nmea_hz"], 0)


class LineFramerCase(unittest.TestCase):
    def test_split_lines_and_no_trailing_fabrication(self):
        framer = LineFramer()
        line = gga()
        self.assertEqual(framer.feed(line[:10].encode()), [])
        self.assertEqual(framer.feed((line[10:] + "\r\n").encode()), [line])
        self.assertEqual(framer.feed(b"$partial"), [])

    def test_overlong_line_and_binary_are_discarded_whole(self):
        framer = LineFramer(limit=100)
        self.assertEqual(framer.feed(b"$" + b"x" * 200 + b"\n"), [])
        self.assertEqual(framer.feed(b"$abc\xff\n"), [])
        self.assertEqual(framer.feed(b"$one*00\n"), ["$one*00"])
        self.assertLessEqual(len(framer.pending), 100)


if __name__ == "__main__":
    unittest.main()
