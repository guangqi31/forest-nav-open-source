"""原始IMU、外参和时间覆盖契约；不启动Isaac、不用真值姿态伪造IMU输出。"""
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from forest_nav_control.sensor_contract import validate_imu_packet,extrinsics,rotation,imu_pose_from_lidar
from forest_nav_control.imu_bridge import InertialSync


class Capture:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(msg)


def test_native_packet_requires_real_continuity():
    values=np.zeros((3,6))
    validate_imu_packet(np.array([10,15,20]),np.array([1,2,3]),values,5)
    with pytest.raises(ValueError):validate_imu_packet(np.array([10,20,25]),np.array([1,2,3]),values,5)
    with pytest.raises(ValueError):validate_imu_packet(np.array([10,15,20]),np.array([1,3,4]),values,5)
    values[0,0]=np.nan
    with pytest.raises(ValueError):validate_imu_packet(np.array([10,15,20]),np.array([1,2,3]),values,5)


def test_mount_transform_preserves_original_world_lidar(monkeypatch):
    import forest_nav_control.sensor_contract as contract
    for settings in [contract.imu_settings(),dict(offset_torso_m=[.1,.2,.3],rotation_xyzw=[0.,2**-.5,0.,2**-.5])]:
        monkeypatch.setattr(contract,'imu_settings',lambda:settings)
        lidar=np.array([1.,2.,3.,0.,0.,np.sin(.3),np.cos(.3)])
        imu=imu_pose_from_lidar(lidar);t,q=extrinsics();point=np.array([2.,-.3,.4])
        a=imu[:3]+rotation(imu[3:])@(rotation(q)@point+t)
        b=lidar[:3]+rotation(lidar[3:])@point
        assert np.allclose(a,b,atol=1e-10)


def test_imu_publication_coverage_duplicates_bad_packet_gaps_and_restart(tmp_path):
    rclpy.init();node=Node('sensor_sync_contract_test')
    def stamp(t):
        sec,ns=divmod(round(t*1e9),1000000000);return Time(sec=sec,nanosec=ns)
    bridge=InertialSync(node,tmp_path/'imu.npz',stamp)
    bridge.pub=Capture();bridge.cloud_pub=Capture();bridge.status_pub=Capture()
    progress=SimpleNamespace(session='first',epoch=0,rewind_sent=None,stamp=2.)
    def packet(sequences,stamps,session='first'):
        values=np.tile([.1,.2,.3,0.,0.,9.81],(len(sequences),1))
        np.savez(bridge.path,session=session,sent_monotonic=99.,period_ns=5000000,
            sequences=np.asarray(sequences,dtype=np.int64),stamps_ns=np.asarray(stamps,dtype=np.int64),measurements=values)
    def cloud(t):return PointCloud2(header=Header(frame_id='lidar_link',stamp=stamp(t)))
    try:
        packet([1,2,3],[1990000000,1995000000,2000000000])
        bridge.poll(progress,True,100.);bridge.queue_cloud(cloud(2.),2.);bridge.flush(progress,True)
        assert len(bridge.pub.messages)==3 and len(bridge.cloud_pub.messages)==1
        m=bridge.pub.messages[-1]
        assert m.orientation_covariance[0]==-1. and m.orientation.w==1.
        assert m.linear_acceleration.z==9.81 and m.angular_velocity.x==.1
        bridge.poll(progress,True,100.);assert len(bridge.pub.messages)==3,'不得重复发布同一IMU样本'
        bridge.queue_cloud(cloud(2.005),2.005);bridge.flush(progress,True)
        assert len(bridge.cloud_pub.messages)==1,'IMU未覆盖点云时间时必须等待'
        progress.stamp=2.005;packet([1,2,3,4],[1990000000,1995000000,2000000000,2005000000])
        bridge.poll(progress,True,100.);bridge.flush(progress,True)
        assert len(bridge.cloud_pub.messages)==2
        bridge.path.write_bytes(b'bad npz');progress.stamp=2.01
        bridge.poll(progress,True,100.);bridge.flush(progress,True)
        assert bridge.last_ns==2005000000 and json.loads(bridge.status_pub.messages[-1].data)['ready']
        # 跨包缺样后重新对齐新数据，不锁住后续点云。
        progress.stamp=2.02;packet([6,7],[2015000000,2020000000])
        bridge.poll(progress,True,100.);bridge.queue_cloud(cloud(2.02),2.02);bridge.flush(progress,True)
        assert not bridge.fault and bridge.gap_count==1 and len(bridge.cloud_pub.messages)==3
        assert json.loads(bridge.status_pub.messages[-1].data)['ready']
        progress.session='second';progress.stamp=1.
        packet([1],[1000000000],session='second');bridge.poll(progress,True,100.)
        bridge.queue_cloud(cloud(1.),1.);bridge.flush(progress,True)
        assert not bridge.fault and bridge.count==1 and bridge.synced_count==1
        bridge.flush(progress,False)
        assert json.loads(bridge.status_pub.messages[-1].data)['ready'],'无墙钟过期拦截'
        progress.session='third';progress.stamp=1700000000.001
        packet([1],[1700000000000000001],session='third');bridge.poll(progress,True,100.)
        exact=bridge.pub.messages[-1].header.stamp
        assert exact.sec==1700000000 and exact.nanosec==1,'纳秒不能经过浮点秒损失低位'
        message=PointCloud2(header=Header(frame_id='lidar_link',stamp=Time(sec=1700000000,nanosec=1)))
        bridge.queue_cloud(message,0.);bridge.flush(progress,True)
        assert bridge.last_cloud_ns==1700000000000000001
        # IMU继续新鲜、激光超过两个采样周期没更新时，整体同步状态也必须失效。
        progress.stamp=1700000000.501
        packet([101],[1700000000500000001],session='fourth');progress.session='fourth'
        bridge.poll(progress,True,100.);bridge.last_cloud_ns=1700000000000000001;bridge.synced_count=1
        bridge.flush(progress,True)
        assert json.loads(bridge.status_pub.messages[-1].data)['ready'],'测量年龄只报告，不否决数据流'

    finally:node.destroy_node();rclpy.shutdown()
