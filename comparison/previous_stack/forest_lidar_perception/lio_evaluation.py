"""独立评估LIO；真值仅在本文件使用，不进入估计器或地图。

只用第一对同时刻位姿确定一个固定SE(3)对齐，不用整条轨迹拟合来掩盖漂移。
上游秒转整数纳秒使用截断，可能损失1纳秒；只允许2纳秒数值误差，不配对相邻扫描。
"""
import numpy as np
from forest_nav_control.sensor_contract import rotation, imu_pose_from_lidar


def evaluate(estimated, truth):
    truth_by_time={int(row[0]):imu_pose_from_lidar(row[1:]) for row in truth}
    pairs=[];max_skew=0
    for row in estimated:
        stamp=int(row[0])
        matched=next((stamp+d for d in (0,-1,1,-2,2) if stamp+d in truth_by_time),None)
        if matched is not None:
            pairs.append((stamp,np.asarray(row[1:]),truth_by_time[matched]))
            max_skew=max(max_skew,abs(matched-stamp))
    if len(pairs)<2:return dict(passed=False,paired_poses=len(pairs),reason='缺少同时刻的估计/真值位姿'),np.empty((0,9))
    _,e0,g0=pairs[0]
    align_r=rotation(g0[3:])@rotation(e0[3:]).T
    align_t=g0[:3]-align_r@e0[:3]
    rows=[];previous=None;distance=0.
    for stamp,est,gt in pairs:
        position=align_r@est[:3]+align_t
        error=np.linalg.norm(position-gt[:3])
        angle=np.arccos(np.clip((np.trace((align_r@rotation(est[3:])).T@rotation(gt[3:]))-1)/2,-1,1))
        if previous is not None:distance+=np.linalg.norm(gt[:3]-previous)
        previous=gt[:3]
        rows.append([stamp,*position,*gt[:3],error,float(np.degrees(angle))])
    rows=np.asarray(rows)
    rmse=float(np.sqrt(np.mean(rows[:,7]**2)));end=float(rows[-1,7])
    return dict(passed=bool(len(pairs)>=40 and rmse<.5 and end<.75),paired_poses=len(pairs),
        position_rmse_m=rmse,position_max_error_m=float(rows[:,7].max()),end_position_error_m=end,
        orientation_rmse_deg=float(np.sqrt(np.mean(rows[:,8]**2))),
        truth_path_length_m=float(distance),truth_displacement_m=float(np.linalg.norm(pairs[-1][2][:3]-g0[:3])),
        duration_sim_sec=(pairs[-1][0]-pairs[0][0])*1e-9,max_timestamp_numeric_skew_ns=max_skew,
        alignment='仅首对位姿确定固定SE3，无全轨迹拟合，无尺度修正'),rows
