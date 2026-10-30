"""Actual released Noetic driver + actual sensor_msgs serialization tests.

The adapter has no publishers/serial access, so these run without a master.
Host-only runs skip explicitly; Noetic amd64/ARM64 runs must execute them.
"""
import io
import math
import unittest
from unittest import mock

try:
    import rospy
    from libnmea_navsat_driver.driver import RosNMEADriver
    from sensor_msgs.msg import NavSatFix
    ROS_IMPORT_ERROR = None
except ImportError as error:
    ROS_IMPORT_ERROR = str(error)

from cup_gnss.core import GnssModel
from cup_gnss.upstream_nmea import NmeaFixAdapter


def sentence(body):
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return '${}*{:02X}'.format(body, checksum)


def gga(epoch,quality=4,talker='GN',geoid='-4.5'):
    return sentence(','.join([talker+'GGA',epoch,'3112.0000','N','12130.0000','E',
                              str(quality),'18','0.8','18.2','M',geoid,'M','1.0','0001']))


def qualified(quality=4,talker='GN',geoid='-4.5'):
    model=GnssModel(source='serial')
    model.ingest({'kind':'transport','connected':True},1000,100)
    for index in range(2):
        raw=gga('12000{}.00'.format(index),quality,talker,geoid)
        model.ingest({'v':1,'kind':'nmea','session':'TEST_ONLY','seq':index,
                      'captured_at':1000+index,'_received_wall':1000+index,
                      '_received_mono':100+index,'nmea':raw},1000+index,100+index)
    return raw,model.snapshot(101)


@unittest.skipIf(ROS_IMPORT_ERROR is not None,'Noetic dependency unavailable: '+str(ROS_IMPORT_ERROR))
class ReleasedDriverMessages(unittest.TestCase):
    def test_no_extra_publishers_parameters_or_device_open(self):
        with mock.patch.object(rospy,'Publisher',side_effect=AssertionError('extra publisher')):
            with mock.patch.object(rospy,'get_param',side_effect=AssertionError('parameter access')):
                adapter=NmeaFixAdapter()
                self.assertIsInstance(adapter.driver,RosNMEADriver)
                raw,snapshot=qualified()
                with mock.patch.object(adapter.driver,'add_sentence',wraps=adapter.driver.add_sentence) as add:
                    fix=adapter.to_fix(raw,snapshot,rospy.Time(123456,987654321),'gnss_link')
                    self.assertEqual(add.call_count,1)
        self.assertIsInstance(fix,NavSatFix)
        self.assertEqual(fix.header.stamp,rospy.Time(123456,987654321))
        self.assertEqual(fix.header.frame_id,'gnss_link')
        self.assertAlmostEqual(fix.latitude,31.2)
        self.assertAlmostEqual(fix.longitude,121.5)
        self.assertAlmostEqual(fix.altitude,13.7)
        self.assertEqual(fix.position_covariance_type,NavSatFix.COVARIANCE_TYPE_UNKNOWN)
        self.assertEqual(fix.position_covariance,[0.]*9)
        self.assertEqual(adapter.generated_count,1)
        self.assertEqual(adapter.driver.time_ref_pub.messages,[])
        stream=io.BytesIO();fix.serialize(stream)
        received=NavSatFix().deserialize(stream.getvalue())
        self.assertEqual(received.header.stamp,fix.header.stamp)
        self.assertAlmostEqual(received.altitude,13.7)

    def test_q4_q5_share_ros_status_but_not_quality_or_fresh_fixed_policy(self):
        adapter=NmeaFixAdapter()
        for quality in (4,5):
            raw,snapshot=qualified(quality)
            fix=adapter.to_fix(raw,snapshot,rospy.Time(123,quality),'gnss_link')
            self.assertEqual(fix.status.status,2)
            self.assertEqual(snapshot['diagnostics']['gga_quality'],quality)
            self.assertEqual(fix.position_covariance_type,0)
            self.assertEqual(snapshot['diagnostics']['fix_type'],'RTK fixed' if quality==4 else 'RTK float')

    def test_talker_and_height_compatibility_are_explicit(self):
        adapter=NmeaFixAdapter()
        for talker,service in (('GP',1),('GL',2),('GA',8),('GB',4),('BD',4),('GN',0)):
            raw,snapshot=qualified(talker=talker)
            fix=adapter.to_fix(raw,snapshot,rospy.Time(12,2),'gnss_link')
            self.assertEqual(fix.status.service,service)
        raw,snapshot=qualified(geoid='')
        fix=adapter.to_fix(raw,snapshot,rospy.Time(12,3),'gnss_link')
        self.assertTrue(math.isnan(fix.altitude))

    def test_invalid_policy_does_not_republish_upstream_gbas_as_valid(self):
        adapter=NmeaFixAdapter()
        raw,snapshot=qualified()
        snapshot['fix'].update(valid=False,status=-1,latitude=float('nan'),longitude=float('nan'),altitude=float('nan'))
        fix=adapter.to_fix(raw,snapshot,rospy.Time(12,4),'gnss_link')
        self.assertEqual(fix.status.status,-1)
        self.assertTrue(math.isnan(fix.latitude))
        self.assertEqual(adapter.generated_count,0)

    def test_wrong_associated_raw_sentence_is_refused(self):
        adapter=NmeaFixAdapter()
        raw,snapshot=qualified()
        with self.assertRaises(ValueError):
            adapter.to_fix(raw[:-1],snapshot,rospy.Time(12,5),'gnss_link')
        with self.assertRaises(ValueError):
            adapter.to_fix(raw.replace('3112.0000','3113.0000'),snapshot,rospy.Time(12,5),'gnss_link')


if __name__=='__main__':
    unittest.main()
