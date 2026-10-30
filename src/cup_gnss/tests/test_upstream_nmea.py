"""Contract regressions for the pinned mature NMEA parser adapter."""
import hashlib
import json
import math
from pathlib import Path
import unittest
from unittest import mock

from cup_gnss import upstream_nmea as adapter


def sentence(body):
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return '${}*{:02X}'.format(body, checksum)


def gga(epoch='120000.00', quality=4, talker='GN', lat='3112.0000', ns='N',
        lon='12130.0000', ew='E', altitude='18.2', geoid='-4.5', diff='1.0'):
    return sentence(','.join([talker+'GGA', epoch, lat, ns, lon, ew,
                              str(quality), '18', '0.8', altitude, 'M', geoid,
                              'M', diff, '0001']))


class UpstreamNmeaContract(unittest.TestCase):
    def test_actual_pinned_checksum_and_parser_are_called(self):
        raw = gga()
        with mock.patch.object(adapter.checksum, 'check_nmea_checksum',
                               wraps=adapter.checksum.check_nmea_checksum) as check:
            with mock.patch.object(adapter.parser, 'parse_nmea_sentence',
                                   wraps=adapter.parser.parse_nmea_sentence) as parse:
                parsed = adapter.parse_nmea(raw)
        self.assertTrue(check.called)
        self.assertTrue(parse.called)
        self.assertEqual(parsed['parser_backend'], 'nmea_navsat_driver/0.6.1')
        self.assertAlmostEqual(parsed['latitude'], 31.2)
        self.assertAlmostEqual(parsed['longitude'], 121.5)
        self.assertAlmostEqual(parsed['altitude'], 13.7)
        self.assertEqual(parsed['differential_age_s'], 1)
        self.assertEqual(parsed['epoch'], 43200)
        self.assertEqual(parsed['raw'], raw)

    def test_fixed_float_metadata_remain_distinct(self):
        fixed, floating = adapter.parse_nmea(gga()), adapter.parse_nmea(gga(quality=5))
        self.assertEqual(fixed['quality'], 4)
        self.assertEqual(floating['quality'], 5)
        self.assertNotIn('position_covariance', fixed)

    def test_no_fix_blank_utc_is_a_real_fresh_sentence(self):
        parsed = adapter.parse_nmea('$GNGGA,,,,,,0,,,,,,,,*78')
        self.assertEqual(parsed['quality'], 0)
        self.assertIsNone(parsed['epoch'])
        self.assertIsNone(parsed['latitude'])
        self.assertIsNone(parsed['altitude'])

    def test_checksum_framing_partial_and_garbage_rejected(self):
        raw = gga()
        for invalid in (None, b'raw', raw[:-1], raw[:-2]+'00', raw+'\r\n',
                        raw+'x', raw[:30], '\x00'+raw, '$'+('a'*1030)):
            with self.subTest(invalid=invalid):
                self.assertIsNone(adapter.parse_nmea(invalid))

    def test_coordinate_units_hemispheres_and_bounds(self):
        parsed = adapter.parse_nmea(gga(ns='S', ew='W'))
        self.assertAlmostEqual(parsed['latitude'], -31.2)
        self.assertAlmostEqual(parsed['longitude'], -121.5)
        for key, value in (('lat', '3199.0'), ('lat', '9000.01'),
                           ('lon', '18000.01'), ('ns', ''), ('lat', '-3112.0')):
            parsed = adapter.parse_nmea(gga(**{key: value}))
            self.assertIsNone(parsed['latitude' if key in ('lat','ns') else 'longitude'])
        self.assertEqual(adapter.parse_nmea(gga(lat='0000.0', lon='00000.0'))['latitude'], 0)

    def test_talker_normalization_does_not_change_record_or_input(self):
        for talker in ('GP', 'GL', 'GA', 'GB', 'BD', 'GD', 'GN', 'IN'):
            raw = gga(talker=talker)
            parsed = adapter.parse_nmea(raw)
            self.assertEqual(parsed['talker'], talker)
            self.assertEqual(parsed['raw'], raw)
            self.assertAlmostEqual(parsed['longitude'], 121.5)
            normalized = adapter.upstream_sentence(raw)
            self.assertTrue(adapter.checksum.check_nmea_checksum(normalized))

    def test_measurement_date_never_invented_from_gga_at_midnight(self):
        before, after = adapter.parse_nmea(gga('235959.90')), adapter.parse_nmea(gga('000000.10'))
        self.assertAlmostEqual(before['epoch'], 86399.9)
        self.assertAlmostEqual(after['epoch'], .1)
        for parsed in (before, after):
            self.assertNotIn('measurement_unix_time_s', parsed)
            self.assertEqual(parsed['stamp_semantics'], 'receipt; GGA time-of-day has no date')
        for invalid in ('240000', '126000', '120060', 'aa0000'):
            self.assertIsNone(adapter.parse_nmea(gga(invalid))['epoch'])

    def test_rmc_date_retained_but_not_gga_date_or_ros_stamp(self):
        raw = sentence('GNRMC,235959.90,A,3112.0,N,12130.0,E,0.0,0.0,171026,,,D,S')
        parsed = adapter.parse_nmea(raw)
        self.assertEqual(parsed['status'], 'A')
        self.assertEqual(parsed['rmc_date'], '171026')
        self.assertEqual(parsed['mode'], 'D')
        self.assertEqual(parsed['navigation_status'], 'S')
        self.assertNotIn('measurement_unix_time_s', parsed)

    def test_height_units_and_missing_geoid_are_not_assumed(self):
        self.assertIsNone(adapter.parse_nmea(gga(geoid=''))['altitude'])
        self.assertIsNone(adapter.parse_nmea(gga(altitude='nan'))['altitude'])
        raw = sentence(gga()[1:-3].replace(',18.2,M,', ',18.2,F,'))
        self.assertIsNone(adapter.parse_nmea(raw)['altitude'])
        self.assertIsNone(adapter.parse_nmea(gga(diff='-1'))['differential_age_s'])

    def test_receiver_gst_errors_are_preserved_without_made_up_gga_association(self):
        raw = sentence('GNGST,120000.00,1.2,1.3,1.1,45,0.2,0.3,0.4')
        parsed = adapter.parse_nmea(raw)
        self.assertEqual(parsed['utc_time'], '120000.00')
        self.assertEqual(parsed['epoch'], 43200)
        self.assertEqual(parsed['receiver_error_fields']['lat_std_dev'], .2)
        self.assertEqual(parsed['receiver_error_fields']['lon_std_dev'], .3)
        self.assertEqual(parsed['covariance_source'], 'receiver GST; not associated to a GGA')
        self.assertNotIn('position_covariance', parsed)

    def test_malformed_valid_checksum_never_produces_a_position(self):
        for raw in (sentence('GNGGA,120000.00,3112.0,N,12130.0,E,4'),
                    sentence('GNGGA,120000.00,3112.0,N,12130.0,E,abc,18,0.8,18.2,M,-4.5,M,1,0001')):
            parsed = adapter.parse_nmea(raw)
            self.assertTrue(parsed is None or parsed.get('latitude') is None)

    def test_upstream_copy_manifest_matches_immutable_bytes(self):
        root = Path(adapter.__file__).parent
        record = json.loads((root/'NMEA_UPSTREAM.json').read_text())
        self.assertEqual(record['version'], '0.6.1')
        self.assertEqual(record['commit'], '371f3c9709f424879df76cf6cf4324805ef130b8')
        for name, expected in record['files'].items():
            self.assertEqual(hashlib.sha256((root/name).read_bytes()).hexdigest(), expected)


if __name__ == '__main__':
    unittest.main()
