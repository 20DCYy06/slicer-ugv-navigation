#!/usr/bin/env python3
"""FAST-LIO world->IMU 投影为起始底盘原点的二维里程计，不发布TF。"""
import numpy as np
import yaml
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix, quaternion_from_euler, euler_from_matrix


def main():
    rospy.init_node('slicer_lio_projector')
    c=yaml.safe_load(open(rospy.get_param('~config_file'),encoding='utf-8'));s=c['sensors']
    T_BL=quaternion_matrix(s['base_to_laser']['quaternion']);T_BL[:3,3]=s['base_to_laser']['xyz']
    T_IL=np.eye(4);T_IL[:3,:3]=np.array(s['lidar_to_imu_rotation']).reshape(3,3);T_IL[:3,3]=s['lidar_to_imu_translation']
    T_IB=T_IL@np.linalg.inv(T_BL)
    pub=rospy.Publisher('/lio/odom',Odometry,queue_size=5);origin=[None]
    def callback(msg):
        p=msg.pose.pose;q=p.orientation
        T_WI=quaternion_matrix([q.x,q.y,q.z,q.w]);T_WI[:3,3]=[p.position.x,p.position.y,p.position.z]
        T_WB=T_WI@T_IB
        if origin[0] is None:
            yaw=euler_from_matrix(T_WB)[2];T=quaternion_matrix(quaternion_from_euler(0,0,yaw));T[:3,3]=T_WB[:3,3]
            origin[0]=np.linalg.inv(T)
        T_OB=origin[0]@T_WB;yaw=euler_from_matrix(T_OB)[2];out=Odometry()
        out.header.stamp=msg.header.stamp;out.header.frame_id=c['frames']['odom'];out.child_frame_id=c['frames']['base']
        out.pose.pose.position.x,out.pose.pose.position.y=map(float,T_OB[:2,3])
        q=quaternion_from_euler(0,0,yaw)
        out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=q
        for index in range(6):out.pose.covariance[index*6+index]=1e6
        for index,var in zip((0,7,35),s['lio_pose_variance']):out.pose.covariance[index]=var
        pub.publish(out)
    rospy.Subscriber('/Odometry',Odometry,callback,queue_size=5);rospy.spin()


if __name__=='__main__':main()
