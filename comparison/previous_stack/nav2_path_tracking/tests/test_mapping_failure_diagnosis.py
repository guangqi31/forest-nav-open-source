"""故障诊断：真实ROS节点回调和UDP接口，受控时间，无需再启动Isaac。

当前测试检查真实停更保护与三个修复目标。测试通过不等于通过真机物理
制动验收。TF条件在夹具中固定为有效，
目的是单独检查消息新鲜度、控制许可和UDP零指令传播。
"""
import importlib
import json
import os
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
import rclpy
from rclpy.time import Time
from std_msgs.msg import String,Header,Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2


class Capture:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(msg)


class Rig:
    def state(self):
        # 使用真实SimEndpoint编码和本地UDP传输，实际调用SimBridge解码与看门狗。
        self.endpoint.publish_state(self.sim[0],(0.,0.,0.),(0.,0.,0.),ready=True)
        self.bridge._command(self.command);self.bridge._permit(Bool(data=True))
        self.bridge._tick()
        self.guard.on_bridge(self.bridge.status_pub.messages[-1])
        self.endpoint.read_command()
    def map_status(self):
        self.mapper.tick();message=self.mapper.status.messages[-1]
        self.guard.on_mapping(message);return json.loads(message.data)
    def progress(self):
        self.lidar.progress.path.write_text(json.dumps(dict(stamp=self.sim[0],sent_monotonic=self.wall[0],session=self.session)))
    def source(self):
        self.progress();self.scan_sequence+=1
        np.savez(self.lidar.file,sent_monotonic=self.wall[0],stamp=self.sim[0],sequence=self.scan_sequence,session=self.session,
            points=np.array([[0.,0.,0.]],dtype=np.float32),world_points=np.array([[0.,0.,0.]],dtype=np.float32),
            sensor_pose=np.array([0.,0.,1.,0.,0.,0.,1.]),body_pose=np.array([0.,0.,1.,0.,0.,0.,1.]))
        self.lidar.tick();self.mapper.on_lidar(self.lidar.status.messages[-1])
    def cloud(self,stamp,points):
        fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
                for i,name in enumerate(('x','y','z','intensity'))]
        msg=point_cloud2.create_cloud(Header(frame_id='map',stamp=Time(seconds=stamp).to_msg()),fields,points)
        self.mapper.on_terrain(msg)
    def guard_fresh(self):
        self.guard.on_exploration(String(data=json.dumps(dict(phase='exploring'))))
    def stop_check(self,expected):
        self.guard_fresh();self.guard.tick()
        status=json.loads(self.guard.status_pub.messages[-1].data)
        assert not status['control_permitted'] and status['reason_code']==expected
        assert self.cancel_calls,'没有发起取消'
        # 将真实安全监视输出交给真实桥接回调，再从真实UDP端验证零指令。
        self.bridge._permit(self.guard.permit_pub.messages[-1]);self.bridge._command(self.command)
        self.bridge._tick();received=self.endpoint.read_command()
        assert received==(0.,0.,0.)
        return status
    def record(self,name,**fields):
        directory=os.environ.get('FOREST_NAV_DIAGNOSTIC_RESULTS')
        if directory:
            path=Path(directory);path.mkdir(parents=True,exist_ok=True)
            (path/(name+'.json')).write_text(json.dumps(dict(case=name,passed=True,**fields),ensure_ascii=False,indent=2)+'\n')


@pytest.fixture
def rig(monkeypatch,tmp_path):
    modules={key:importlib.import_module('forest_nav_control.'+name) for key,name in
             [('lidar','lidar_bridge'),('map','terrain_costmap'),('bridge','bridge_node'),('guard','task_guard'),('endpoint','sim_endpoint')]}
    r=Rig();r.wall=[100.];r.sim=[10.];r.session='fault-diagnosis';r.cancel_calls=[];r.scan_sequence=0
    fake=SimpleNamespace(monotonic=lambda:r.wall[0],monotonic_ns=lambda:round(r.wall[0]*1e9),time=lambda:1000.)
    for module in modules.values():monkeypatch.setattr(module,'time',fake)
    monkeypatch.setenv('FOREST_NAV_MAP_MODE','lidar');monkeypatch.setenv('FOREST_NAV_MAP_ARCHIVE',str(tmp_path/'maps'))
    monkeypatch.setenv('FOREST_LIDAR_FRAME_FILE',str(tmp_path/'lidar.npz'))
    r.endpoint=modules['endpoint'].SimEndpoint(state_port=0,command_port=0);r.endpoint.session=r.session
    port=r.endpoint.socket.getsockname()[1]
    rclpy.init(args=['--ros-args','-p','state_port:=0','-p',f'command_port:={port}','-p','require_navigation_permit:=true'])
    r.bridge=modules['bridge'].SimBridge();r.guard=modules['guard'].TaskGuard();r.mapper=modules['map'].TerrainCostmap();r.lidar=modules['lidar'].LidarBridge()
    r.endpoint.destination=r.bridge.socket.getsockname()
    r.bridge.status_pub=Capture();r.guard.permit_pub=Capture();r.guard.status_pub=Capture();r.guard.mission_pub=Capture()
    r.mapper.status=Capture();r.lidar.status=Capture();r.lidar.diag=Capture()
    monkeypatch.setattr(r.mapper,'get_clock',lambda:SimpleNamespace(now=lambda:Time(seconds=r.sim[0])))
    r.mapper.initialized=True;r.mapper.saved_revision=99999;r.mapper.last_save_wall=100.
    r.guard.map_valid=True;r.guard.active=True;r.guard.goal_id='fixture';r.guard.session=r.session;r.guard.state='navigating'
    r.guard.tf=SimpleNamespace(can_transform=lambda *args:True)
    monkeypatch.setattr(r.guard,'tf_fresh',lambda:True)
    monkeypatch.setattr(r.guard,'request_cancel',lambda:r.cancel_calls.append(r.wall[0]) or True)
    r.command=Twist();r.command.linear.x=.2
    try:
        r.source();r.cloud(10.,[[0.,0.,0.,0.]])
        r.state();r.map_status();r.guard_fresh();r.guard.tick()
        assert r.guard.permit_pub.messages[-1].data
        r.bridge._permit(r.guard.permit_pub.messages[-1]);r.bridge._command(r.command);r.bridge._tick()
        assert r.endpoint.read_command()==(.2,0.,0.)
        yield r
    finally:
        r.lidar.destroy_node();r.mapper.destroy_node();r.guard.destroy_node();r.bridge.destroy_node()
        r.endpoint.close();rclpy.shutdown()


def test_lidar_stops_while_simulation_continues(rig):
    r=rig;r.wall[0]+=1.1;r.sim[0]+=.5;r.state();r.progress()
    # 旧帧保留，步进心跳继续；测量已落后0.5仿真秒，不能靠心跳续命。
    r.lidar.tick();r.mapper.on_lidar(r.lidar.status.messages[-1]);mapping=r.map_status()
    status=r.stop_check('mapping_not_ready')
    r.record('lidar_stopped',failed_checks=mapping['diagnostics']['failed_checks'],status=status,udp_command_zero=True)


def test_cmu_stops_but_lidar_and_robot_continue(rig):
    r=rig;r.wall[0]+=2.01;r.sim[0]+=.7;r.state();r.source()
    mapping=r.map_status();assert mapping['diagnostics']['failed_checks']==['terrain_sim_fresh']
    r.stop_check('mapping_not_ready')
    r.record('cmu_stopped',failed_checks=mapping['diagnostics']['failed_checks'],udp_command_zero=True)


def test_simulation_pause_repeated_timestamp_does_not_refresh_state(rig):
    r=rig;r.wall[0]+=.51
    # 发送新序号但重复仿真时间，不能伪装成机器人状态仍在推进。
    r.state();assert r.bridge.state_received==100.
    r.map_status();status=r.stop_check('robot_state_timeout')
    r.record('simulation_paused',state_age_sec=json.loads(r.bridge.status_pub.messages[-1].data)['state_age_sec'],status=status,udp_command_zero=True)


def test_lidar_bridge_lost_while_simulation_continues(rig):
    r=rig;r.wall[0]+=1.01;r.sim[0]+=.3;r.state()
    mapping=r.map_status()
    assert mapping['diagnostics']['failed_checks']==['lidar_status_fresh']
    r.stop_check('mapping_not_ready');r.record('lidar_bridge_lost',udp_command_zero=True)


def test_map_node_lost_while_robot_continues(rig):
    r=rig;r.wall[0]+=2.01;r.sim[0]+=.4;r.state()
    r.stop_check('mapping_not_ready');r.record('map_node_lost',udp_command_zero=True)


def test_state_bridge_lost_endpoint_independently_expires_command(rig):
    r=rig;r.wall[0]+=.351
    # 不再调用SimBridge；接收端自身失联看门狗必须起作用。
    assert r.endpoint.read_command()==(0.,0.,0.)
    r.wall[0]=100.51;r.guard_fresh();r.guard.tick()
    status=json.loads(r.guard.status_pub.messages[-1].data)
    assert not status['control_permitted'] and status['reason_code']=='robot_state_timeout'
    r.record('state_bridge_lost',endpoint_timeout_sec=.35,guard_timeout_sec=.5,udp_command_zero=True)


def test_future_cloud_preserves_recent_good_history_but_never_extends_it(rig):
    r=rig;r.wall[0]+=.01
    r.cloud(10.12,[[1.,0.,0.,0.]])
    assert r.mapper.grid.last_stamp==10. and r.mapper.terrain_received==100.
    mapping=r.map_status();assert mapping['ready']
    assert mapping['diagnostics']['last_rejection']['reason']=='CMU地形点云过期或来自未来'
    # 持续没有新的有效地形时，旧地图寿命耗尽，仍然必须停止。
    r.wall[0]+=.2;r.sim[0]=10.7;r.state();r.source()
    r.cloud(10.82,[[1.,0.,0.,0.]])
    mapping=r.map_status();assert not mapping['ready']
    assert r.mapper.grid.last_stamp==10. and r.mapper.terrain_received==100.
    r.stop_check('mapping_not_ready')
    r.record('single_future_cloud_preserves_history',prior_good_stamp=10.,later_sim_time=10.7,expired_without_valid_update=True,udp_command_zero=True)


def test_late_cloud_never_erases_global_map_without_clock_rewind(rig):
    r=rig;r.wall[0]+=.1;r.sim[0]=20.01;r.source()
    r.cloud(20.,[[0.,0.,0.,0.],[1.,0.,.4,.4]])
    initial=int((r.mapper.grid.cost>=0).sum());assert initial==2
    # 迟到帧仍在允许的测量年龄内，但顺序更旧，必须忽略且不能清图。
    r.cloud(19.45,[[3.,0.,0.,0.]])
    assert r.sim[0]==20.01 and r.mapper.grid.last_stamp==20.
    assert int((r.mapper.grid.cost>=0).sum())==2
    old_cell=int((0.-r.mapper.grid.origin)/r.mapper.grid.resolution)
    assert r.mapper.grid.cost[old_cell,old_cell]==0
    assert r.mapper.terrain_rejections==0
    r.record('late_cloud_preserves_history',sim_time_before_and_after=20.01,
        input_stamps=[20.,19.45],last_accepted_stamp=20.,known_cells_before=initial,known_cells_after=2)


def test_actual_clock_rewind_clears_map_and_invalidates_old_task(rig):
    r=rig;generation=r.mapper.reset_generation
    r.sim[0]=1.;r.wall[0]+=.1
    mapping=r.map_status()
    assert not mapping['ready'] and (r.mapper.grid.cost==-1).all()
    assert r.mapper.reset_generation>generation
    assert r.mapper.terrain_received is None
    r.guard.map_generation=generation
    # 即使后续健康地图已恢复，旧任务也不能越过重置继续运行。
    r.mapper.status.messages.clear();r.source();r.cloud(1.,[[0.,0.,0.,0.]])
    mapping=r.map_status();assert mapping['ready']
    # 机器人状态在真实系统会因同会话时钟回退被拒收；此处直接提供恢复后健康状态，
    # 单独验证地图代次变更能撤销旧任务。
    r.guard.on_bridge(String(data=json.dumps(dict(ready=True,session=r.session,sim_time=1.,pose=[0.,0.,0.]))))
    r.guard_fresh();r.guard.tick()
    status=json.loads(r.guard.status_pub.messages[-1].data)
    assert not status['control_permitted'] and status['reason_code']=='map_reset'
    r.record('actual_clock_rewind',history_cleared=True,old_task_invalidated=True)
