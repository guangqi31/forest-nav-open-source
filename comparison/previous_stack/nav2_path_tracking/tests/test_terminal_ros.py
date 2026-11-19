"""真实 ROS Action/话题测试，运动学夹具不代表真实 G1 接触性能。"""
import json
import math
import threading
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from std_msgs.msg import String, Bool
from forest_nav_control.terminal_node import TerminalNode
from forest_nav_control.unknown_navigation import UnknownNavigation


def wait(future, timeout=15.):
    deadline=time.monotonic()+timeout
    while not future.done() and time.monotonic()<deadline:time.sleep(.02)
    assert future.done(), '测试夹具等待超时；不改变生产任务时限'
    return future.result()


class Plant(Node):
    def __init__(self):
        super().__init__('terminal_control_test_plant')
        self.group=ReentrantCallbackGroup();self.pose=[0.,0.,0.];self.cmd=(0.,0.,0.);self.stamp=1.
        self.quiet=0.;self.last_cmd=0.;self.outputs=[]
        self.healthy=True;self.localization_healthy=True
        self.native_cancels=0
        self.state_pubs={key:self.create_publisher(String,topic,10) for key,topic in (
            ('bridge','/navigation/bridge_status'),('guard','/navigation/task_status'),('mapping','/navigation/mapping_status'))}
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.maps=[self.create_publisher(OccupancyGrid,topic,qos) for topic in
                   ('/traversability_map','/global_costmap/costmap','/local_costmap/costmap')]
        self.odom=self.create_publisher(Odometry,'/odom',10)
        self.stop=self.create_publisher(Bool,'/navigation/stop_requested',10)
        self.sub=self.create_subscription(Twist,'/cmd_vel',self.command,10,callback_group=self.group)
        self.native=ActionServer(self,NavigateToPose,'/navigate_to_pose',self.drive,
                                cancel_callback=lambda h:CancelResponse.ACCEPT,callback_group=self.group)
        self.planner=ActionServer(self,ComputePathToPose,'/compute_path_to_pose',self.plan,callback_group=self.group)
        self.client=ActionClient(self,NavigateToPose,'/navigation/navigate_to_goal',callback_group=self.group)
        self.direct=ActionClient(self,NavigateToPose,'/navigation/terminal_pose',callback_group=self.group)
        self.timer=self.create_timer(.05,self.tick,callback_group=self.group)

    def command(self,msg):
        self.cmd=(msg.linear.x,msg.linear.y,msg.angular.z);self.last_cmd=time.monotonic();self.outputs.append(self.cmd)

    def tick(self):
        if time.monotonic()-self.last_cmd>.3:self.cmd=(0.,0.,0.)
        vx,_,wz=self.cmd
        self.pose[0]+=vx*math.cos(self.pose[2])*.05;self.pose[1]+=vx*math.sin(self.pose[2])*.05;self.pose[2]+=wz*.05
        self.stamp+=.05;self.quiet=self.quiet+.05 if not any(self.cmd) else 0.
        loc=dict(settled=self.quiet>1.,measured_tilt_rad=0.,mode='standing' if self.quiet>1. else 'walking')
        values=dict(bridge=dict(ready=True,pose=self.pose,sim_time=self.stamp,locomotion=loc,
                               localization_source="fast_lio",localization_ready=self.localization_healthy,
                               localization_stamp=self.stamp,localization_age_sec=0.),
                    guard=dict(control_permitted=self.healthy,pose=self.pose),
                    mapping=dict(ready=self.healthy,session='fixture',reset_generation=0,map_to_odom=[0.,0.,0.],known_cells=400))
        for key,pub in self.state_pubs.items():pub.publish(String(data=json.dumps(values[key])))
        for i,pub in enumerate(self.maps):
            grid=OccupancyGrid();grid.header.frame_id='odom' if i==2 else 'map'
            grid.header.stamp.sec=int(self.stamp);grid.header.stamp.nanosec=int((self.stamp-int(self.stamp))*1e9)
            grid.info.width=grid.info.height=50;grid.info.resolution=.2
            grid.info.origin.position.x=grid.info.origin.position.y=-5.;grid.info.origin.orientation.w=1.
            grid.data=[0]*2500;pub.publish(grid)
        msg=Odometry();msg.header.frame_id='odom';msg.child_frame_id='base_link';msg.pose.pose.orientation.w=1.
        msg.twist.twist.linear.x=vx;msg.twist.twist.angular.z=wz;self.odom.publish(msg)

    def plan(self,h):
        result=ComputePathToPose.Result();result.path.poses=[h.request.goal];h.succeed();return result

    def drive(self,h):
        # 故意停在目标前0.32米，验收是否由真实终点Action完成剩余纠偏。
        p=h.request.pose.pose;self.pose=[p.position.x-.32,p.position.y,0.]
        end=time.monotonic()+.6
        while time.monotonic()<end:
            if h.is_cancel_requested:
                self.native_cancels+=1;h.canceled();return NavigateToPose.Result()
            time.sleep(.02)
        h.succeed();return NavigateToPose.Result()


def test_upper_action_terminal_hold_reacquire_and_stop(tmp_path,monkeypatch):
    monkeypatch.setenv('FOREST_TERRAIN_SCENE','fixture')
    monkeypatch.setenv('FOREST_NAV_TERMINAL_CONTROL','1');monkeypatch.setenv('FOREST_NAV_DUAL_POLICY','1')
    monkeypatch.setenv('FOREST_NAV_MISSION_RESULTS',str(tmp_path))
    rclpy.init();plant=Plant();terminal=TerminalNode();manager=UnknownNavigation()
    executor=MultiThreadedExecutor(num_threads=10)
    for node in (plant,terminal,manager):executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        assert plant.client.wait_for_server(timeout_sec=3.)
        time.sleep(.5)
        request=NavigateToPose.Goal();request.pose.header.frame_id='map';request.pose.pose.position.x=.7;request.pose.pose.orientation.w=1.
        handle=wait(plant.client.send_goal_async(request));assert handle.accepted
        response=wait(handle.get_result_async(),20.)
        assert response.status==4 and terminal.controller.verified
        assert plant.native_cancels==1  # 终点邻域内确认取消旧控制，才接管速度。
        assert manager.state['parking_status']=='verified'
        assert any(0<cmd[0]<.12 for cmd in plant.outputs)
        fixed=terminal.controller.target
        plant.pose[0]-=.3
        end=time.monotonic()+3.
        while time.monotonic()<end and (terminal.controller.verified or not any(terminal.command)):time.sleep(.02)
        assert terminal.controller.target==fixed and not terminal.controller.verified
        assert any(terminal.command)
        # 统一停止必须撤销完成后的保持；许可仍健康时也不能追原目标。
        plant.stop.publish(Bool(data=True));time.sleep(.4)
        assert terminal.controller is None and terminal.command==(0.,0.,0.)
        plant.pose[0]-=.2;time.sleep(.4)
        assert terminal.command==(0.,0.,0.)
        # 新的显式目标允许重新取得控制；执行中的Action取消必须确认结束。
        request.pose.pose.position.x=plant.pose[0]+.3
        active=wait(plant.direct.send_goal_async(request));assert active.accepted
        time.sleep(.2);wait(active.cancel_goal_async())
        assert wait(active.get_result_async()).status==5
        time.sleep(.1);assert terminal.controller is None and terminal.command==(0.,0.,0.)
        # 数据/许可恢复不能复活已经故障的纠偏任务。
        request.pose.pose.position.x=plant.pose[0]+.3
        active=wait(plant.direct.send_goal_async(request));assert active.accepted
        time.sleep(.2);plant.healthy=False
        time.sleep(.2);assert not terminal.fault
        plant.healthy=True
        assert wait(active.get_result_async()).status==4
    finally:
        manager.epoch+=1
        with terminal.lock:terminal.clear('test_shutdown')
        executor.shutdown(timeout_sec=5.);thread.join(5.)
        for node in (manager,terminal,plant):node.destroy_node()
        rclpy.shutdown()


def test_state_age_tilt_and_ready_flags_do_not_latch_terminal_fault():
    from types import SimpleNamespace as NS
    fake=NS(data=dict(bridge=dict(pose=[0.,0.,0.],sim_time=10.,ready=False,
        localization_ready=False,locomotion=dict(measured_tilt_rad=1.)),velocity=(0.,0.,0.)),
        received={},goal_context=None)
    assert TerminalNode.health(fake,time.monotonic())==''
    assert TerminalNode.health(fake,time.monotonic()+100.)==''


def test_missing_computation_input_waits_without_time_based_fault():
    from types import SimpleNamespace as NS
    fake=NS(data={})
    assert TerminalNode.health(fake,time.monotonic())=='waiting_pose'
    fake.data['bridge']=dict(pose=[0.,0.,0.])
    assert TerminalNode.health(fake,time.monotonic())=='waiting_feedback'


def test_localization_report_does_not_abort_terminal(monkeypatch):
    monkeypatch.setenv('FOREST_TERRAIN_SCENE','fixture')
    rclpy.init();plant=Plant();terminal=TerminalNode()
    executor=MultiThreadedExecutor(num_threads=6)
    for node in (plant,terminal):executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        assert plant.direct.wait_for_server(timeout_sec=3.)
        deadline=time.monotonic()+3.
        while time.monotonic()<deadline:
            with terminal.lock:
                if not terminal.health(time.monotonic()):break
            time.sleep(.02)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
        goal.pose.pose.position.x=.7;goal.pose.pose.orientation.w=1.
        active=wait(plant.direct.send_goal_async(goal));assert active.accepted
        time.sleep(.2);plant.localization_healthy=False
        time.sleep(.2);assert not terminal.fault
        plant.localization_healthy=True
        assert wait(active.get_result_async()).status==4
    finally:
        with terminal.lock:terminal.clear('test_shutdown')
        executor.shutdown(timeout_sec=5.);thread.join(5.)
        for node in (terminal,plant):node.destroy_node()
        rclpy.shutdown()
