"""Prevent adjacent-epoch diagnostics from authorizing old static coordinates."""
import contextlib
import csv
import io
import json
import tempfile
import types
import importlib.util
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location("epoch_evidence", Path(__file__).resolve().parents[1] / "tools" / "rtk_evidence.py")
evidence = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(evidence)


class EpochEvidenceCase(unittest.TestCase):
    def samples(self, epoch="120001.00", quality="4"):
        raw = {"utc": "120001.00", "quality": 4, "latitude": 31.0, "longitude": 118.0,
               "altitude_ellipsoid_m": 20.0, "differential_age_s": 1.0}
        fix = {"status": 2, "latitude": 31.0, "longitude": 118.0, "altitude": 20.0}
        diagnostic = {"statuses": [{"name": "cup_gnss/receiver", "values": {
            "gga_utc_time": epoch, "gga_quality": quality, "session": "current-session",
            "fix_valid": "true", "invalid_reason": "", "fresh_rtk_usable": "true",
            "data_age_s": "0.1", "gga_age_s": "0.1", "fix_epoch_age_s": "0.1",
            "effective_differential_age_s": "1.1", "rtcm_serial_write_age_s": "0.1"}}]}
        return raw, fix, diagnostic

    def test_later_valid_epoch_cannot_authorize_earlier_static_gga(self):
        # Stationary coordinates match exactly; later recovery must not validate
        # an earlier epoch whose own invalid fix/diagnostics were delayed/lost.
        self.assertEqual(evidence.qualify(*self.samples(epoch="120002.00"), advancing=True),
                         "diagnostic_gga_epoch_mismatch")

    def test_unknown_epoch_cannot_authorize_a_trajectory_point(self):
        self.assertEqual(evidence.qualify(*self.samples(epoch=""), advancing=True),
                         "diagnostic_gga_epoch_mismatch")

    def test_other_raw_quality_cannot_authorize_fixed_trajectory(self):
        self.assertEqual(evidence.qualify(*self.samples(quality="5"), advancing=True),
                         "diagnostic_gga_quality_mismatch")

    def test_analysis_uses_same_epoch_diagnostic_within_window(self):
        def sentence(second):
            body = "GNGGA,1200{:02d}.00,3100.000,N,11800.000,E,4,20,0.7,18,M,2,M,1,0001".format(second)
            checksum = 0
            for byte in body.encode("ascii"):
                checksum ^= byte
            return "$" + body + "*{:02X}".format(checksum)
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            records = []
            def add(source, stamp, **values):
                records.append(dict(source=source, record=len(records), receipt_monotonic_s=stamp,
                                    receipt_utc="fixture", phase="static", **values))
            add("nmea", 100.0, raw=sentence(0))
            add("diagnostics", 100.99, **self.samples(epoch="120000.00")[2])
            add("nmea", 101.0, raw=sentence(1))
            add("fix", 101.01, **self.samples()[1])
            add("diagnostics", 101.8, **self.samples()[2])
            (run / "messages.jsonl").write_text("".join(json.dumps(r) + "\n" for r in records))
            (run / "capture.json").write_text(json.dumps({"started_monotonic_s": 99.0}))
            args = types.SimpleNamespace(input=directory, timeout=2.5, match_window=0.5,
                                         diagnostic_window=1.5, max_differential_age=5.0)
            with contextlib.redirect_stdout(io.StringIO()):
                evidence.analyze(args)
            with next(run.glob("analysis_*/trajectory.csv")).open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[1]["accepted"], "True")
            self.assertEqual(rows[1]["diagnostic_record"], "4")

    def test_same_epoch_with_equivalent_decimal_format_is_accepted(self):
        self.assertEqual(evidence.qualify(*self.samples(epoch="120001.000"), advancing=True), "")


if __name__ == "__main__":
    unittest.main()
