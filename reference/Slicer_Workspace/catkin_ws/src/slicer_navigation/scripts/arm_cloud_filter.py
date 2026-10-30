#!/usr/bin/env python3
"""Remove only high, body-fixed arm echoes before 3-D cloud becomes a 2-D scan.

The raw registered cloud remains untouched for LIO and diagnostics. A narrow
measured XY envelope plus a measured minimum arm height avoids erasing low
obstacles in front of the gripper.
"""
import copy
import math

import numpy as np
import rospy
import tf
import yaml
from sensor_msgs.msg import PointCloud2, PointField
from tf.transformations import quaternion_matrix


def mask_arm_points(msg, translation, rotation, rear_x, front_x, half_width, min_z):
    fields = {field.name: field for field in msg.fields}
    if not all(name in fields and fields[name].datatype == PointField.FLOAT32
               and fields[name].count == 1 for name in ('x', 'y', 'z')):
        raise ValueError('Cloud requires scalar FLOAT32 x/y/z fields')
    if msg.point_step < max(fields[name].offset + 4 for name in ('x', 'y', 'z')):
        raise ValueError('Invalid PointCloud2 field offsets')
    data = bytearray(msg.data)
    dtype = '>f4' if msg.is_bigendian else '<f4'
    shape = (msg.height, msg.width)
    strides = (msg.row_step, msg.point_step)
    xyz = [np.ndarray(shape, dtype=dtype, buffer=data, offset=fields[name].offset,
                      strides=strides) for name in ('x', 'y', 'z')]
    matrix = quaternion_matrix(rotation)
    # Transform from source cloud frame into base_link. The IMU/body frame is
    # not at the ground-projected chassis centre, so raw x/y/z cannot be masked.
    bx = matrix[0, 0]*xyz[0] + matrix[0, 1]*xyz[1] + matrix[0, 2]*xyz[2] + translation[0]
    by = matrix[1, 0]*xyz[0] + matrix[1, 1]*xyz[1] + matrix[1, 2]*xyz[2] + translation[1]
    bz = matrix[2, 0]*xyz[0] + matrix[2, 1]*xyz[1] + matrix[2, 2]*xyz[2] + translation[2]
    mask = (np.isfinite(bx) & np.isfinite(by) & np.isfinite(bz) &
            (bx >= rear_x) & (bx <= front_x) &
            (np.abs(by) <= half_width) & (bz >= min_z))
    for coord in xyz:
        coord[mask] = math.nan
    result = copy.copy(msg)
    result.data = bytes(data)
    result.is_dense = False
    return result, int(np.count_nonzero(mask))


def main():
    rospy.init_node('slicer_arm_cloud_filter')
    with open(rospy.get_param('~config_file'), encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    envelope = config['navigation']['travel_envelope']
    min_z = float(config['sensors']['arm_self_filter_min_z'])
    rear_x, front_x = float(envelope['rear_x']), float(envelope['front_x'])
    half_width = float(envelope['half_width'])
    if not (-1 < rear_x < 0 < front_x < 1 and 0 < half_width < 1 and 0 < min_z < 2):
        raise ValueError('Invalid arm self-filter envelope')
    listener = tf.TransformListener()
    pub = rospy.Publisher('/cloud_nav_filtered', PointCloud2, queue_size=1)

    def callback(msg):
        try:
            translation, rotation = listener.lookupTransform(config['frames']['base'],
                                                              msg.header.frame_id, rospy.Time(0))
            filtered, count = mask_arm_points(msg, translation, rotation,
                                               rear_x, front_x, half_width, min_z)
        except (tf.Exception, ValueError) as exc:
            # Never pass unfiltered cloud into navigation if TF/format is lost.
            rospy.logerr_throttle(5, 'Arm cloud filter unavailable: %s', exc)
            return
        rospy.loginfo_throttle(30, 'Arm self-filter removed %d high points from latest cloud', count)
        pub.publish(filtered)

    rospy.Subscriber('/cloud_registered_body', PointCloud2, callback, queue_size=1,
                     tcp_nodelay=True)
    rospy.spin()


if __name__ == '__main__':
    main()
