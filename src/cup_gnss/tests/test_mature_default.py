import unittest
from cup_gnss.core import parse_nmea

class MatureDefaultTests(unittest.TestCase):
    def test_default_validated_gga_retains_raw_quality_and_names_upstream(self):
        body='GNGGA,235959.00,3156.28660023,N,11847.09918862,E,5,30,0.6,17.7102,M,2.2110,M,1.0,3854'
        check=0
        for byte in body.encode('ascii'):check^=byte
        record=parse_nmea('$'+body+'*%02X'%check)
        self.assertEqual(record.get('parser_backend'),'nmea_navsat_driver/0.6.1')
        self.assertEqual(record['quality'],5)
        self.assertEqual(record['epoch'],86399.)
        self.assertAlmostEqual(record['altitude'],19.9212)

    def test_default_ntrip_constructor_does_not_build_legacy_http(self):
        from unittest.mock import patch
        from cup_gnss.upstream_ntrip import UpstreamNtripClient
        with patch('cup_gnss.ntrip.build_request',side_effect=AssertionError('legacy HTTP builder must not run')):
            client=UpstreamNtripClient('/TEST_ONLY-not-opened',lambda:None,lambda frame:len(frame),host='127.0.0.1',port=2101,mountpoint='TEST_ONLY')
        self.assertEqual(client.snapshot()['backend'],'ntrip_client')
