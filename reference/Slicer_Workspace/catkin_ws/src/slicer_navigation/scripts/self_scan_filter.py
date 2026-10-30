#!/usr/bin/env python3
"""在base_link切片后的scan中屏蔽车身矩形内回波，保留车头近场障碍。"""
import math
import copy
import yaml
import rospy
from sensor_msgs.msg import LaserScan


def main():
    rospy.init_node('slicer_scan_filter')
    c=yaml.safe_load(open(rospy.get_param('~config_file'),encoding='utf-8'))
    half_x=c['chassis']['body_length']/2+c['sensors']['self_filter_padding']
    half_y=c['chassis']['body_width']/2+c['sensors']['self_filter_padding']
    front_half_angle=math.radians(float(c['sensors']['front_fov_deg'])/2.0)
    if not 0 < front_half_angle <= math.pi:
        raise ValueError('front_fov_deg must be within (0, 360]')
    pub=rospy.Publisher('/scan',LaserScan,queue_size=2)
    front_pub=rospy.Publisher('/scan_front',LaserScan,queue_size=2)
    def callback(msg):
        if msg.header.frame_id!=c['frames']['base']:
            rospy.logerr_throttle(5,'Scan self-filter requires base_link scan; refusing wrong-frame cloud')
            return
        ranges=list(msg.ranges)
        for i,r in enumerate(ranges):
            theta=msg.angle_min+i*msg.angle_increment
            if math.isfinite(r) and abs(r*math.cos(theta))<=half_x and abs(r*math.sin(theta))<=half_y:
                # NaN: do not raytrace-clear through the self-occluded body.
                ranges[i]=float('nan')
        msg.ranges=ranges
        pub.publish(msg)
        front=copy.copy(msg)
        front.ranges=[r if abs(math.atan2(math.sin(msg.angle_min+i*msg.angle_increment),
                                         math.cos(msg.angle_min+i*msg.angle_increment)))<=front_half_angle
                      else float('nan') for i,r in enumerate(ranges)]
        front_pub.publish(front)
    rospy.Subscriber('/scan_raw',LaserScan,callback,queue_size=2);rospy.spin()


if __name__=='__main__':main()
