#!/usr/bin/env python3
"""TF仅从统一配置生成。body是FAST-LIO IMU坐标；绝不硬编码旧底盘外参。"""
import numpy as np
import yaml
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_matrix, quaternion_from_matrix


def main():
    rospy.init_node('slicer_geometry')
    c=yaml.safe_load(open(rospy.get_param('~config_file'),encoding='utf-8'));s=c['sensors'];f=c['frames']
    transforms=[]
    def add(parent,child,xyz,quat):
        m=TransformStamped();m.header.stamp=rospy.Time.now();m.header.frame_id=parent;m.child_frame_id=child
        m.transform.translation.x,m.transform.translation.y,m.transform.translation.z=map(float,xyz)
        m.transform.rotation.x,m.transform.rotation.y,m.transform.rotation.z,m.transform.rotation.w=map(float,quat)
        transforms.append(m)
    for parent,child,key in ((f['base'],f['laser'],'base_to_laser'),(f['laser'],f['arm'],'laser_to_arm')):
        add(parent,child,s[key]['xyz'],s[key]['quaternion'])
    T_BL=quaternion_matrix(s['base_to_laser']['quaternion']);T_BL[:3,3]=s['base_to_laser']['xyz']
    T_IL=np.eye(4);T_IL[:3,:3]=np.asarray(s['lidar_to_imu_rotation']).reshape(3,3);T_IL[:3,3]=s['lidar_to_imu_translation']
    T_BI=T_BL@np.linalg.inv(T_IL)
    add(f['base'],'body',T_BI[:3,3],quaternion_from_matrix(T_BI))
    broadcaster=tf2_ros.StaticTransformBroadcaster();broadcaster.sendTransform(transforms)
    rospy.spin()


if __name__=='__main__':main()
