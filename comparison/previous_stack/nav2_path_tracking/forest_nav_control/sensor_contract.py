"""激光/IMU共享的纯数据契约；不导入ROS或Isaac，双方可独立使用。"""
import json
from pathlib import Path
import numpy as np

CONFIG=Path(__file__).resolve().parents[2]/'forest_lidar_perception/config'


def imu_settings():return json.loads((CONFIG/'imu.json').read_text())


def rotation(q):
    q=np.asarray(q,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or not np.isclose(np.linalg.norm(q),1.,atol=1e-4):
        raise ValueError('安装四元数必须是单位xyzw四元数')
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def quaternion_multiply(a,b):
    av,aw=np.asarray(a)[:3],float(a[3]);bv,bw=np.asarray(b)[:3],float(b[3])
    return np.r_[aw*bv+bw*av+np.cross(av,bv),aw*bw-np.dot(av,bv)]


def extrinsics():
    """返回T_imu_lidar：p_imu=R_imu_lidar @ p_lidar+t_imu_lidar。"""
    imu=imu_settings();lidar=json.loads((CONFIG/'lidar.json').read_text())
    q=np.asarray(imu['rotation_xyzw'],dtype=float);q_inv=q*np.array([-1.,-1.,-1.,1.])
    t=rotation(q).T@(np.asarray(lidar['offset_torso_m'])-np.asarray(imu['offset_torso_m']))
    return t,q_inv  # 激光坐标轴与父torso轴平行，当前激光没有额外安装旋转。


def imu_pose_from_lidar(lidar_pose):
    """仅供原有真值TF使用；绝不将这个姿态填入原始IMU测量。"""
    t,q=extrinsics();q_i=quaternion_multiply(lidar_pose[3:],q*np.array([-1.,-1.,-1.,1.]))
    return np.r_[np.asarray(lidar_pose[:3])-rotation(q_i)@t,q_i]


def validate_imu_packet(stamps,sequences,measurements,period_ns):
    stamps=np.asarray(stamps);sequences=np.asarray(sequences);values=np.asarray(measurements)
    if stamps.dtype.kind not in 'iu' or sequences.dtype.kind not in 'iu' or stamps.ndim!=1 or sequences.shape!=stamps.shape:
        raise ValueError('IMU时间和序号必须为等长整数一维数组')
    if values.shape!=(len(stamps),6) or not len(stamps) or not np.isfinite(values).all():
        raise ValueError('IMU测量必须是非空有限N×6数组')
    if np.any(stamps<=0) or np.any(sequences<=0) or int(period_ns)<=0:
        raise ValueError('IMU时间、序号和周期必须为正数')
    if len(stamps)>1 and (np.any(np.diff(stamps)!=int(period_ns)) or np.any(np.diff(sequences)!=1)):
        raise ValueError('IMU缓冲不连续或乱序')
