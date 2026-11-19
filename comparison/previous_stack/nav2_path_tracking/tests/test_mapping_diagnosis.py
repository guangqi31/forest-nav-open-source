"""地图健康修复回归：慢仿真和单个坏帧不误停，旧测量不因心跳获得续命。"""
import importlib
import json
from types import SimpleNamespace
import numpy as np
import rclpy
from rclpy.time import Time
from std_msgs.msg import String,Header
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2


class Capture:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(msg)


def test_slow_simulation_keeps_healthy_without_republishing_old_cloud(monkeypatch,tmp_path):
    lidar_module=importlib.import_module('forest_nav_control.lidar_bridge')
    map_module=importlib.import_module('forest_nav_control.terrain_costmap')
    frame=tmp_path/'latest.npz';progress=tmp_path/'lidar_progress.json'
    monkeypatch.setenv('FOREST_LIDAR_FRAME_FILE',str(frame));monkeypatch.setenv('FOREST_NAV_MAP_ARCHIVE',str(tmp_path/'maps'))
    wall=[100.];sim=[5.42];fake=SimpleNamespace(monotonic=lambda:wall[0],time=lambda:1000.)
    monkeypatch.setattr(lidar_module,'time',fake);monkeypatch.setattr(map_module,'time',fake)
    rclpy.init();lidar=lidar_module.LidarBridge();mapping=map_module.TerrainCostmap()
    lidar.status=Capture();lidar.diag=Capture();lidar.pub=Capture();lidar.registered=Capture();mapping.status=Capture()
    monkeypatch.setattr(mapping,'get_clock',lambda:SimpleNamespace(now=lambda:Time(seconds=sim[0])))
    def heartbeat():
        progress.write_text(json.dumps(dict(stamp=sim[0],sent_monotonic=wall[0],session='replay')))
        lidar.tick();mapping.on_lidar(lidar.status.messages[-1]);mapping.tick()
    try:
        np.savez(frame,points=np.array([[0.,0.,0.]],dtype=np.float32),world_points=np.array([[0.,0.,0.]],dtype=np.float32),
            sensor_pose=np.array([0.,0.,1.,0.,0.,0.,1.]),body_pose=np.array([0.,0.,1.,0.,0.,0.,1.]),
            sent_monotonic=100.,stamp=5.42,session='replay',sequence=23)
        heartbeat()
        fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z','intensity'))]
        mapping.on_terrain(point_cloud2.create_cloud(Header(frame_id='map',stamp=Time(seconds=5.42).to_msg()),fields,[[0.,0.,0.,0.]]))
        # 连续步进：2.4个实际秒只前进0.16仿真秒，还没到下一次0.2秒采样。
        # 原来雷达1墙钟秒、地形2墙钟秒检查都会误停；现在旧点云仍只发布一次。
        for step in range(1,13):
            wall[0]=100.+step*.2;sim[0]=5.42+step*.16/12
            heartbeat();assert json.loads(lidar.status.messages[-1].data)['ready']
            assert json.loads(mapping.status.messages[-1].data)['ready']
        assert len(lidar.pub.messages)==len(lidar.registered.messages)==1
        assert mapping.terrain_received==100. and mapping.grid.last_stamp==5.42
        # 旧文件被一个坏写入覆盖时，最后有效测量仍按原时间老化，不能立即失效。
        frame.write_bytes(b'invalid npz');wall[0]+=.1;sim[0]+=.01
        heartbeat();assert json.loads(lidar.status.messages[-1].data)['ready']
        assert json.loads(mapping.status.messages[-1].data)['ready']
        # 仿真继续超过两个采样周期却仍没有新雷达，必须撤销ready。
        wall[0]+=.1;sim[0]=5.92;heartbeat()
        assert not json.loads(lidar.status.messages[-1].data)['ready']
        assert not json.loads(mapping.status.messages[-1].data)['ready']
        assert len(lidar.pub.messages)==1
    finally:
        mapping.destroy_node();lidar.destroy_node();rclpy.shutdown()


def test_progress_clock_requires_actual_advance_and_handles_rewind(tmp_path):
    from forest_nav_control.observation_health import ProgressClock
    path=tmp_path/'progress.json';clock=ProgressClock(path)
    def write(stamp,sent):path.write_text(json.dumps(dict(stamp=stamp,sent_monotonic=sent,session='same')))
    write(10.,100.);assert clock.poll(100.)
    write(10.,100.6);assert not clock.poll(100.6),'墙钟心跳不能伪装成时钟推进'
    write(10.02,100.61);assert clock.poll(100.61)
    assert not clock.poll(101.2),'真正失联必须被发现'
    write(1.,101.21);assert clock.poll(101.21) and clock.epoch==1
    assert clock.rewind_sent==101.21
