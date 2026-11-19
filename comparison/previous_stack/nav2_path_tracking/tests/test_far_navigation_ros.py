"""FAR适配层的真实ROS Action测试；运动学夹具不代表G1物理验证。"""
import math
import threading
import time
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,ActionClient,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav2_msgs.action import NavigateToPose,FollowPath
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import Twist,PoseStamped
from std_msgs.msg import String,Bool
from visibility_graph_msg.msg import ForestGoal,ForestRoute
from forest_nav_control.far_navigation import FarNavigation
from forest_nav_control.terminal_node import TerminalNode


def wait(future,timeout=10.):
    end=time.monotonic()+timeout
    while not future.done() and time.monotonic()<end:time.sleep(.02)
    assert future.done(),'test fixture timeout'
    return future.result()


class Plant(Node):
    def __init__(self):
        super().__init__('far_mission_test_plant');self.group=ReentrantCallbackGroup()
        self.pose=[0.,0.,0.];self.cmd=(0.,0.,0.);self.stamp=1.;self.quiet=0.
        self.task='';self.goal=0.;self.canceled=False;self.latest=None;self.fail_lio=False
        self.received_requests=[];self.path_goals=[];self.last_cmd=time.monotonic()
        self.rpub=self.create_publisher(ForestRoute,'/far/route',10)
        self.sub=self.create_subscription(ForestGoal,'/far/request',self.request,10,callback_group=self.group)
        self.csub=self.create_subscription(Twist,'/cmd_vel',self.command,10,callback_group=self.group)
        self.cpub=self.create_publisher(Twist,'/cmd_vel_nav2',10)
        self.odom=self.create_publisher(Odometry,'/odom',10)
        self.bridge=self.create_publisher(String,'/navigation/bridge_status',10)
        self.mapping=self.create_publisher(String,'/navigation/mapping_status',10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pubs=[self.create_publisher(OccupancyGrid,t,qos) for t in ('/traversability_map','/local_costmap/costmap')]
        self.follow=ActionServer(self,FollowPath,'/follow_path',self.drive,
            cancel_callback=lambda h:CancelResponse.ACCEPT,callback_group=self.group)
        self.client=ActionClient(self,NavigateToPose,'/navigation/navigate_to_goal',callback_group=self.group)
        self.timer=self.create_timer(.05,self.tick,callback_group=self.group)

    def request(self,msg):
        self.received_requests.append(msg)
        if msg.cancel:
            if msg.task_id==self.task:self.canceled=True
        else:self.task=msg.task_id;self.goal=msg.goal.x;self.canceled=False

    def command(self,msg):
        self.cmd=(msg.linear.x,msg.linear.y,msg.angular.z);self.last_cmd=time.monotonic()

    def drive(self,h):
        self.latest=h;self.path_goals.append(h.request.path)
        while rclpy.ok():
            if h.is_cancel_requested:
                h.canceled();return FollowPath.Result()
            if self.latest is not h:
                h.abort();return FollowPath.Result()
            x=h.request.path.poses[-1].pose.position.x
            if abs(x-self.pose[0])<.05:
                h.succeed();return FollowPath.Result()
            cmd=Twist();cmd.linear.x=.4;self.cpub.publish(cmd);time.sleep(.05)
        h.abort();return FollowPath.Result()

    def tick(self):
        if time.monotonic()-self.last_cmd>.3:self.cmd=(0.,0.,0.)
        vx,vy,wz=self.cmd;self.pose[0]+=vx*.05;self.pose[2]+=wz*.05
        self.stamp+=.05;sec=int(self.stamp);nano=int((self.stamp-sec)*1e9)
        self.quiet=self.quiet+.05 if not any(self.cmd) else 0.
        bridge=dict(ready=True,pose=list(self.pose),sim_time=self.stamp,localization_ready=not self.fail_lio,
            localization_source='fast_lio',localization_stamp=self.stamp,localization_age_sec=0.,
            locomotion=dict(settled=self.quiet>=1.,measured_tilt_rad=0.))
        self.bridge.publish(String(data=json.dumps(bridge)))
        self.mapping.publish(String(data=json.dumps(dict(ready=True,session='test',reset_generation=0,map_to_odom=[0.,0.,0.]))))
        for i,pub in enumerate(self.map_pubs):
            grid=OccupancyGrid();grid.header.frame_id='map' if i==0 else 'odom';grid.header.stamp.sec=sec;grid.header.stamp.nanosec=nano
            grid.info.width=grid.info.height=50;grid.info.resolution=.2;grid.info.origin.orientation.w=1.
            grid.info.origin.position.x=grid.info.origin.position.y=-2.;grid.data=[0]*2500;pub.publish(grid)
        odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_link';odom.header.stamp.sec=sec
        odom.twist.twist.linear.x=vx;odom.twist.twist.angular.z=wz;self.odom.publish(odom)
        route=ForestRoute();route.task_id=self.task;route.ready=True;route.canceled=self.canceled
        route.header.frame_id=route.path.header.frame_id='lio_odom';route.header.stamp.sec=sec;route.header.stamp.nanosec=nano
        route.path.header.stamp=route.header.stamp
        route.valid=bool(self.task) and not self.canceled
        for x in (self.pose[0],getattr(self,'route_limit',self.goal)):
            p=PoseStamped();p.pose.position.x=float(x);p.pose.orientation.w=1.;route.path.poses.append(p)
        self.rpub.publish(route)


def test_replace_cancel_terminal_and_localization_fault(monkeypatch,tmp_path):
    monkeypatch.setenv('FOREST_NAV_MISSION_RESULTS',str(tmp_path))
    monkeypatch.setenv('FOREST_NAV_PLANNER','far');monkeypatch.setenv('FOREST_TERRAIN_SCENE','fixture')
    rclpy.init();plant=Plant();manager=FarNavigation();terminal=TerminalNode()
    executor=MultiThreadedExecutor(num_threads=12)
    for n in (plant,manager,terminal):executor.add_node(n)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    def send(x):
        g=NavigateToPose.Goal();g.pose.header.frame_id='map';g.pose.pose.position.x=float(x);g.pose.pose.orientation.w=1.
        h=wait(plant.client.send_goal_async(g));assert h.accepted;return h
    try:
        assert plant.client.wait_for_server(timeout_sec=3.)
        time.sleep(.6)
        first=send(3.);time.sleep(.6)
        second=send(plant.pose[0]+.3)
        assert wait(first.get_result_async()).status==6
        assert wait(second.get_result_async(),15.).status==4
        assert manager.state['parking_status']=='verified'
        # New goal must release the terminal hold and accept a fresh path.
        third=send(3.);time.sleep(.5);wait(third.cancel_goal_async())
        assert wait(third.get_result_async()).status==5
        time.sleep(.2);assert terminal.command==(0.,0.,0.)
        fourth=send(3.);time.sleep(.5);plant.fail_lio=True
        time.sleep(.4)
        assert not fourth.get_result_async().done() and manager.control_active
        plant.fail_lio=False
        wait(fourth.cancel_goal_async());assert wait(fourth.get_result_async()).status==5
        # Only FAR routes are executed, no calls to ComputePathToPose/NavigateToPose.
        assert plant.path_goals and len({m.task_id for m in plant.received_requests if not m.cancel})>=3
    finally:
        manager.epoch+=1
        try:manager.stop_actions()
        except RuntimeError:pass
        with terminal.lock:terminal.clear('test_shutdown')
        executor.shutdown(timeout_sec=5.);thread.join(5.)
        for n in (manager,terminal,plant):n.destroy_node()
        rclpy.shutdown()


def test_short_far_route_is_forwarded_without_length_gate(monkeypatch,tmp_path):
    monkeypatch.setenv('FOREST_NAV_PLANNER','far');monkeypatch.setenv('FOREST_TERRAIN_SCENE','fixture')
    monkeypatch.setenv('FOREST_NAV_MISSION_RESULTS',str(tmp_path))
    rclpy.init();plant=Plant();plant.route_limit=.2;manager=FarNavigation();terminal=TerminalNode()
    executor=MultiThreadedExecutor(num_threads=10)
    for n in (plant,manager,terminal):executor.add_node(n)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        assert plant.client.wait_for_server(timeout_sec=3.);time.sleep(.6)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.pose.position.x=3.;goal.pose.pose.orientation.w=1.
        h=wait(plant.client.send_goal_async(goal));assert h.accepted
        time.sleep(.7)
        assert plant.path_goals and not h.get_result_async().done()
        wait(h.cancel_goal_async());assert wait(h.get_result_async()).status==5
    finally:
        manager.epoch+=1
        try:manager.stop_actions()
        except RuntimeError:pass
        executor.shutdown(timeout_sec=5.);thread.join(5.)
        for n in (manager,terminal,plant):n.destroy_node()
        rclpy.shutdown()


def test_follow_abort_retries_without_ending_upper_task(monkeypatch,tmp_path):
    monkeypatch.setenv('FOREST_NAV_PLANNER','far');monkeypatch.setenv('FOREST_TERRAIN_SCENE','fixture')
    monkeypatch.setenv('FOREST_NAV_TERMINAL_CONTROL','1');monkeypatch.setenv('FOREST_NAV_MISSION_RESULTS',str(tmp_path))
    original=Plant.drive
    def fail_once(self,h):
        self.attempts=getattr(self,'attempts',0)+1
        if self.attempts==1:
            h.abort();return FollowPath.Result()
        return original(self,h)
    monkeypatch.setattr(Plant,'drive',fail_once)
    rclpy.init();plant=Plant();manager=FarNavigation();terminal=TerminalNode()
    executor=MultiThreadedExecutor(num_threads=10)
    for n in (plant,manager,terminal):executor.add_node(n)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        assert plant.client.wait_for_server(timeout_sec=3.);time.sleep(.4)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.pose.position.x=3.;goal.pose.pose.orientation.w=1.
        h=wait(plant.client.send_goal_async(goal));assert h.accepted
        time.sleep(.8)
        assert plant.attempts>=2 and not h.get_result_async().done()
        wait(h.cancel_goal_async());assert wait(h.get_result_async()).status==5
    finally:
        manager.epoch+=1;manager.stop_actions()
        with terminal.lock:terminal.clear('test_shutdown')
        executor.shutdown(timeout_sec=5.);thread.join(5.)
        for n in (manager,terminal,plant):n.destroy_node()
        rclpy.shutdown()


def test_native_follow_path_can_complete_without_terminal_server(monkeypatch,tmp_path):
    monkeypatch.setenv('FOREST_NAV_PLANNER','far');monkeypatch.setenv('FOREST_TERRAIN_SCENE','fixture')
    monkeypatch.setenv('FOREST_NAV_TERMINAL_CONTROL','0');monkeypatch.setenv('FOREST_NAV_MISSION_RESULTS',str(tmp_path))
    rclpy.init();plant=Plant();plant.cpub=plant.create_publisher(Twist,'/cmd_vel',10);manager=FarNavigation()
    executor=MultiThreadedExecutor(num_threads=8)
    for n in (plant,manager):executor.add_node(n)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        assert plant.client.wait_for_server(timeout_sec=3.);time.sleep(.4)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.pose.position.x=1.;goal.pose.pose.orientation.w=1.
        h=wait(plant.client.send_goal_async(goal));assert h.accepted
        assert wait(h.get_result_async(),10.).status==4
        assert manager.state['parking_status']=='unverified'
    finally:
        manager.epoch+=1;manager.stop_actions()
        executor.shutdown(timeout_sec=5.);thread.join(5.)
        for n in (manager,plant):n.destroy_node()
        rclpy.shutdown()
