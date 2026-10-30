#!/usr/bin/env python3
"""D455深度避障：5Hz采样、5cm体素、真实TF与LIO重力姿态；不写速度话题。"""
# Adapted from Slicer_Workspace/catkin_ws/src/slicer_navigation_without_mapping/
# scripts/depth_obstacles.py, offline snapshot 20261030; original kept unchanged.
# Local change: use the pure frame-time gate; projection/TF behavior is retained.
import threading
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
import rospy,tf,tf2_ros
from tf.transformations import quaternion_matrix,quaternion_from_euler,quaternion_from_matrix,euler_from_matrix
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image,CameraInfo,PointCloud2,PointField
from std_msgs.msg import Header
from depth_geometry import project,filter_points,valid_frame_times


def tf_message(parent,child,T,stamp):
    m=TransformStamped();m.header.stamp=stamp;m.header.frame_id=parent;m.child_frame_id=child
    m.transform.translation.x,m.transform.translation.y,m.transform.translation.z=map(float,T[:3,3])
    q=quaternion_from_matrix(T);m.transform.rotation.x,m.transform.rotation.y,m.transform.rotation.z,m.transform.rotation.w=map(float,q)
    return m


def cloud(points,frame,stamp):
    a=np.asarray(points,dtype='<f4').reshape(-1,3)
    return PointCloud2(header=Header(stamp=stamp,frame_id=frame),height=1,width=len(a),
        fields=[PointField(name=k,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,k in enumerate('xyz')],
        is_bigendian=False,point_step=12,row_step=12*len(a),data=a.tobytes(),is_dense=True)


class DepthObstacles:
    def __init__(self):
        self.p=rospy.get_param('~');p=self.p;self.listener=tf.TransformListener();self.lock=threading.Lock()
        self.info=None;self.tilt=None;self.gravity_stamp=rospy.Time(0)
        self.latest_depth=None;self.processed_stamp=None;self.body_to_base=None;self.camera_to_base=None
        self.static=tf2_ros.StaticTransformBroadcaster();self.dynamic=tf2_ros.TransformBroadcaster()
        T=quaternion_matrix(quaternion_from_euler(*np.deg2rad(p['mount_rpy_deg'])));T[:3,3]=p['mount_xyz_m']
        self.static.sendTransform(tf_message(p['base_frame'],p['mount_frame'],T,rospy.Time.now()))
        self.hits=rospy.Publisher('/online_depth/obstacles',PointCloud2,queue_size=1)
        self.rays=rospy.Publisher('/online_depth/clearing',PointCloud2,queue_size=1)
        rospy.Subscriber(p['info_topic'],CameraInfo,self.on_info,queue_size=1)
        rospy.Subscriber(p['gravity_topic'],Odometry,self.on_gravity,queue_size=1)
        rospy.Subscriber(p['depth_topic'],Image,self.on_depth,queue_size=1,buff_size=2**21)
        # 图像回调只保留最新帧，投影在定时器中执行；负载高时丢弃旧帧而非积压。
        self.timer=rospy.Timer(rospy.Duration(1./p['publish_hz']),self.process_latest)

    def on_info(self,msg):
        with self.lock:self.info=msg

    def on_gravity(self,msg):
        try:
            # body与base_link是刚性安装外参，不必每个里程计回调重复查询TF。
            if self.body_to_base is None:
                _,q=self.listener.lookupTransform(msg.child_frame_id,self.p['base_frame'],rospy.Time(0))
                self.body_to_base=quaternion_matrix(q)
            o=msg.pose.pose.orientation;R=quaternion_matrix([o.x,o.y,o.z,o.w])@self.body_to_base
            # odom->base是原系统二维TF；用LIO完整姿态补回roll/pitch，移除世界yaw。
            yaw=euler_from_matrix(R)[2];R=quaternion_matrix(quaternion_from_euler(0,0,-yaw))@R
            with self.lock:self.tilt=R;self.gravity_stamp=msg.header.stamp
        except tf.Exception as exc:rospy.logwarn_throttle(3,'Depth gravity TF: %s',exc)

    def on_depth(self,msg):
        with self.lock:self.latest_depth=msg

    def process_latest(self,event):
        p=self.p
        with self.lock:msg,info,tilt,gravity_stamp=self.latest_depth,self.info,self.tilt,self.gravity_stamp
        if msg is None or msg.header.stamp==self.processed_stamp:return
        if info is None or tilt is None:
            rospy.logwarn_throttle(3,'Depth waiting for CameraInfo and FAST-LIO gravity pose');return
        now=rospy.Time.now()
        last_stamp=None if self.processed_stamp is None else self.processed_stamp.to_sec()
        if not valid_frame_times(msg.header.stamp.to_sec(),gravity_stamp.to_sec(),now.to_sec(),
                                 p['gravity_max_age_s'],last_stamp):
            rospy.logwarn_throttle(3,'Depth waiting for fresh, ordered, time-aligned LIO gravity pose: depth_age=%.3fs lio_age=%.3fs delta=%.3fs',
                (now-msg.header.stamp).to_sec(),(now-gravity_stamp).to_sec(),(msg.header.stamp-gravity_stamp).to_sec());return
        try:
            if (info.width,info.height)!=(msg.width,msg.height):raise ValueError('Depth/CameraInfo resolution mismatch')
            kind={'16UC1':('u2',p['depth_scale']),'32FC1':('f4',1.)}.get(msg.encoding)
            if kind is None:raise ValueError('Unsupported depth encoding '+msg.encoding)
            dtype=np.dtype(('>' if msg.is_bigendian else '<')+kind[0])
            # 先在原始深度上取样，再转换单位；避免每次拷贝并缩放整幅640x480图像。
            stride=p['pixel_stride']
            depth=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.step,dtype.itemsize))[::stride,::stride].astype(np.float32)*kind[1]
            fx,fy,cx,cy=(info.P[0],info.P[5],info.P[2],info.P[6]) if info.P[0]>0 else (info.K[0],info.K[4],info.K[2],info.K[5])
            if min(fx,fy)<=0:raise ValueError('Invalid depth intrinsics')
            # 相机固定在车身；缓存静态光学外参，动态倾角仍来自当前LIO反馈。
            if self.camera_to_base is None:
                t,q=self.listener.lookupTransform(p['base_frame'],msg.header.frame_id,rospy.Time(0))
                self.camera_to_base=quaternion_matrix(q);self.camera_to_base[:3,3]=t
            T=tilt@self.camera_to_base
            points=project(depth,fx/stride,fy/stride,cx/stride,cy/stride,1,p['min_depth_m'],p['max_depth_m'])
            hits,rays=filter_points(points,T,p['min_obstacle_height'],p['max_obstacle_height'],p['voxel_size_m'])
            # 使用重力校正后的光学TF，costmap中的高度过滤与本节点严格处于同一坐标定义。
            self.dynamic.sendTransform(tf_message(p['base_frame'],p['output_frame'],T,msg.header.stamp))
            self.hits.publish(cloud(hits,p['output_frame'],msg.header.stamp));self.rays.publish(cloud(rays,p['output_frame'],msg.header.stamp))
            self.processed_stamp=msg.header.stamp
            rospy.loginfo_throttle(10,'Depth obstacles=%d clearing=%d (ground excluded from marking)',len(hits),len(rays))
        except (tf.Exception,ValueError,TypeError) as exc:rospy.logwarn_throttle(3,'Depth projection: %s',exc)


if __name__=='__main__':
    rospy.init_node('online_depth_obstacles');node=DepthObstacles();rospy.spin()
