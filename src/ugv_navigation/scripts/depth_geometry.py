"""深度投影/重力方向地面过滤的纯NumPy核心，无ROS和硬件副作用。"""
# Adapted from Slicer_Workspace/catkin_ws/src/slicer_navigation_without_mapping/
# scripts/depth_geometry.py, offline snapshot 20261030; original kept unchanged.
# Local changes: frame-time validation and finite, positive-focal intrinsics.
import numpy as np


def valid_frame_times(depth_stamp,gravity_stamp,now,max_age,processed_stamp=None):
    """时间均为同一ROS时钟的秒值；保留边界，拒绝过远及已处理/倒序帧。"""
    if processed_stamp is not None and depth_stamp<=processed_stamp:return False
    return (abs(now-depth_stamp)<=max_age
            and abs(now-gravity_stamp)<=max_age
            and abs(depth_stamp-gravity_stamp)<=max_age)


def project(depth,fx,fy,cx,cy,stride,minimum,maximum):
    if not np.isfinite([fx,fy,cx,cy]).all() or fx<=0 or fy<=0:
        raise ValueError('Invalid depth intrinsics')
    v,u=np.mgrid[0:depth.shape[0]:stride,0:depth.shape[1]:stride]
    z=depth[::stride,::stride]
    with np.errstate(invalid='ignore'):valid=np.isfinite(z)&(z>=minimum)&(z<=maximum)
    z=z[valid]
    return np.column_stack(((u[valid]-cx)*z/fx,(v[valid]-cy)*z/fy,z))


def voxel(points,size):
    if not len(points):return points.reshape(-1,3)
    _,index=np.unique(np.floor(points/size).astype(np.int32),axis=0,return_index=True)
    return points[np.sort(index)]


def filter_points(points,T_level_optical,minimum_height,maximum_height,size):
    # 在重力对齐坐标中判高度；保留完整合法测距用于clearing，不把地面当作标障碍点。
    level=points@T_level_optical[:3,:3].T+T_level_optical[:3,3]
    mask=(level[:,2]>=minimum_height)&(level[:,2]<=maximum_height)
    return voxel(points[mask],size),voxel(points,size)
