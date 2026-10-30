"""Offline geometry/evidence fixtures; no claim of live-device acceptance."""
import importlib.util
import json
import math
import contextlib
import io
import types
from pathlib import Path
import sys
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "src"))
from cup_gnss.enu import FixedOriginTrack, WGS84_A, geodetic_to_ecef, to_enu


class EnuCase(unittest.TestCase):
    def test_ecef_axis_and_local_axes(self):
        self.assertEqual(geodetic_to_ecef(0, 0, 0), (WGS84_A, 0.0, 0.0))
        origin = {"latitude": 0, "longitude": 0, "altitude_ellipsoid_m": 0}
        self.assertEqual(to_enu(0, 0, 3, origin), (0.0, 0.0, 3.0))
        east, north, up = to_enu(0, math.degrees(1.0 / WGS84_A), 0, origin)
        self.assertAlmostEqual(east, 1, places=7)
        self.assertAlmostEqual(north, 0, places=7)
        self.assertLess(abs(up), 1e-6)
        east, north, up = to_enu(math.degrees(1.0 / 6335439.3272928195), 0, 0, origin)
        self.assertAlmostEqual(north, 1, places=7)
        self.assertAlmostEqual(east, 0, places=7)

    def test_rejected_and_nonfinite_samples_cannot_create_origin(self):
        with tempfile.TemporaryDirectory() as directory:
            track = FixedOriginTrack(directory)
            self.assertFalse(track.add(31, 118, 20, False, "corrections_stale")["accepted"])
            self.assertFalse(track.add(31, 118, float("nan"), True)["accepted"])
            self.assertFalse(Path(directory, "origin.json").exists())
            self.assertFalse(track.add(91, 118, 20, True)["accepted"])
            self.assertIsNone(track.origin)

    def test_disconnect_and_reanalysis_preserve_origin(self):
        with tempfile.TemporaryDirectory() as directory:
            track = FixedOriginTrack(directory)
            self.assertTrue(track.add(31, 118, 20, True, metadata={"record": 7})["accepted"])
            original = Path(directory, "origin.json").read_bytes()
            track.add(31, 118, 20, False, "serial_disconnected")
            recovered = FixedOriginTrack(directory)
            point = recovered.add(31.00001, 118, 20, True)
            self.assertGreater(point["north_m"], 1)
            self.assertEqual(recovered.origin["metadata"]["record"], 7)
            self.assertEqual(Path(directory, "origin.json").read_bytes(), original)

    def test_damaged_origin_refuses_silent_reset(self):
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, "origin.json").write_text('{"version": 99}', encoding="utf-8")
            with self.assertRaises(ValueError):
                FixedOriginTrack(directory)


class EvidenceCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("rtk_evidence", PACKAGE / "tools" / "rtk_evidence.py")
        cls.evidence = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.evidence)

    def diagnostic(self, **values):
        base = {"gga_utc_time": "120001.00", "gga_quality": "4", "fix_valid": "true", "fresh_rtk_usable": "true", "invalid_reason": "",
                "data_age_s": "0.1", "gga_age_s": "0.1", "fix_epoch_age_s": "0.1",
                "effective_differential_age_s": "1.0"}
        base.update(values)
        return {"statuses": [{"name": "cup_gnss/receiver", "values": base}]}

    def test_quality4_does_not_override_expired_corrections(self):
        gga = {"utc": "120001.00", "quality": 4, "latitude": 31.0, "longitude": 118.0, "altitude_ellipsoid_m": 20.0,
               "differential_age_s": 1.0}
        fix = {"status": 2, "latitude": 31.0, "longitude": 118.0, "altitude": 20.0}
        self.assertEqual(self.evidence.qualify(gga, fix, self.diagnostic(), True), "")
        self.assertEqual(self.evidence.qualify(gga, fix, self.diagnostic(fresh_rtk_usable="false"), True), "node_fresh_rtk_unusable")
        self.assertEqual(self.evidence.qualify(gga, fix, self.diagnostic(effective_differential_age_s="6"), True), "differential_age_missing_or_stale")
        self.assertEqual(self.evidence.qualify(gga, fix, self.diagnostic(rtcm_serial_write_age_s="4.9"), True, diagnostic_age_s=0.2), "serial_rtcm_write_stale")
        self.assertEqual(self.evidence.qualify(gga, fix, self.diagnostic(), False), "gga_epoch_not_advancing")
        self.assertEqual(self.evidence.qualify(gga, fix, self.diagnostic(fresh_rtk_usable="unknown"), True), "node_fresh_rtk_unproven")

    def test_bad_raw_or_mismatched_ros_is_not_track(self):
        gga = {"utc": "120001.00", "quality": 4, "latitude": 31.0, "longitude": 118.0, "altitude_ellipsoid_m": 20.0,
               "differential_age_s": 1.0}
        fix = {"status": 2, "latitude": 31.1, "longitude": 118.0, "altitude": 20.0}
        self.assertEqual(self.evidence.qualify(gga, fix, self.diagnostic(), True), "ros_raw_position_mismatch")


    def test_offline_analysis_keeps_stale_fixed_out_and_preserves_reference(self):
        def sentence(second):
            body = "GNGGA,1200{:02d}.00,3100.000,N,11800.000,E,4,20,0.7,18,M,2,M,1,0001".format(second)
            checksum = 0
            for byte in body.encode("ascii"):
                checksum ^= byte
            return "$" + body + "*{:02X}".format(checksum)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            records = []
            for second in range(5):
                stamp = 100.0 + second
                shared = {"receipt_monotonic_s": stamp, "receipt_utc": "fixture", "phase": "rtk_static" if second < 3 else "rtcm_off"}
                for source, values in (("nmea", {"raw": sentence(second), "topic": "/gnss/nmea"}),
                                       ("fix", {"status": 2, "latitude": 31.0, "longitude": 118.0, "altitude": 20.0}),
                                       ("diagnostics", self.diagnostic(gga_utc_time="1200{:02d}.00".format(second), fresh_rtk_usable="true" if second < 3 else "false"))):
                    row = dict(shared, source=source, record=len(records))
                    row.update(values)
                    records.append(row)
            (root / "messages.jsonl").write_text("".join(json.dumps(r) + "\n" for r in records), encoding="utf-8")
            (root / "capture.json").write_text(json.dumps({"started_monotonic_s": 100.0}), encoding="utf-8")
            (root / "markers.jsonl").write_text(json.dumps({"label": "A", "monotonic_s": 101.0, "utc": "fixture"}) + "\n" + json.dumps({"label": "B", "monotonic_s": 102.0, "utc": "fixture"}) + "\n" + json.dumps({"label": "stale", "monotonic_s": 103.0, "utc": "fixture"}) + "\n", encoding="utf-8")
            args = types.SimpleNamespace(input=directory, timeout=2.5, match_window=0.5, diagnostic_window=1.5, max_differential_age=5.0)
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(self.evidence.analyze(args), 0)
            original = (root / "origin.json").read_bytes()
            summary = json.loads(next(root.glob("analysis_*/summary.json")).read_text(encoding="utf-8"))
            marked_csv = next(root.glob("analysis_*/marked_points.csv")).read_text(encoding="utf-8")
            self.assertIn("stale,fixture,False,node_fresh_rtk_unusable", marked_csv)
            self.assertEqual(summary["raw_quality4_ratio"], 1)
            self.assertEqual(summary["fresh_rtk_ratio"], 0.4)
            self.assertEqual(summary["time_to_first_fresh_rtk_s"], 1)
            self.assertEqual(summary["rejection_reasons"]["node_fresh_rtk_unusable"], 2)
            self.assertTrue(next(root.glob("analysis_*/point_distances.csv")).read_text(encoding="utf-8").count("A,B") == 1)
            with contextlib.redirect_stdout(io.StringIO()):
                self.evidence.analyze(args)
            self.assertEqual((root / "origin.json").read_bytes(), original)
            self.assertEqual(len(list(root.glob("analysis_*"))), 2)


if __name__ == "__main__":
    unittest.main()
