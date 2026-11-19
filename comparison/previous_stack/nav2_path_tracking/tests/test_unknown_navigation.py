"""小规模ROS接口夹具：验证任务编排，不代替真实Nav2/Isaac/G1运动验收。"""
import json,os,threading,time
from pathlib import Path
import numpy as np
import pytest
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String,Bool
from nav2_msgs.action import NavigateToPose,ComputePathToPose
from rclpy.action import ActionClient,ActionServer,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_srvs.srv import Trigger
from forest_nav_control.exploration_policy import observation_candidates


def grid(edge=2.):
    m=OccupancyGrid();m.header.frame_id='map';m.info.resolution=.2;m.info.width=m.info.height=50
    m.info.origin.position.x=m.info.origin.position.y=-5.;m.info.origin.orientation.w=1.
    a=np.full((50,50),-1,dtype=np.int8);a[15:35,15:int((edge+5)/.2)]=0;m.data=a.ravel().tolist();return m


def frontiers():return [dict(size=20,points=[[2.1,y] for y in np.arange(-1.9,2.,.2)])]


def test_candidates_retreat_from_unknown_and_disperse():
    m=grid();out=observation_candidates(m,m,frontiers(),(0,0),(3.5,0))
    assert out and all(.8<=np.hypot(x,y)<=4 for x,y,_ in out)
    assert all(x<=1.45 for x,y,_ in out)
    assert all(np.hypot(a[0]-b[0],a[1]-b[1])>=.8 for n,a in enumerate(out) for b in out[n+1:])
    detailed=observation_candidates(m,m,frontiers(),(0,0),(3.5,0),with_metadata=True)
    assert detailed and all(set(entry)=={
        'point','score','robot_distance_m','frontier_distance_m','frontier_gain'
    } for entry in detailed)


def test_exclusion_and_inflated_obstacles():
    m=grid();out=observation_candidates(m,m,frontiers(),(0,0),(3.5,0))
    next_out=observation_candidates(m,m,frontiers(),(0,0),(3.5,0),out)
    assert all(np.hypot(x-a,y-b)>.8 for x,y,_ in next_out for a,b,_ in out)
    occupied=grid();occupied.data=[99 if x>=0 else -1 for x in occupied.data]
    assert not observation_candidates(m,occupied,frontiers(),(0,0),(3.5,0))


class Fixture(Node):
    def __init__(self):
        super().__init__('unknown_navigation_fixture');self.group=ReentrantCallbackGroup()
        self.raw=grid();self.pose=[0.,0.,0.];self.healthy=True;self.reveal=True;self.slow=False;self.calls=[];self.reset_generation=0
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.rawpub=self.create_publisher(OccupancyGrid,'/traversability_map',qos)
        self.infpub=self.create_publisher(OccupancyGrid,'/global_costmap/costmap',qos)
        self.guardpub=self.create_publisher(String,'/navigation/task_status',10)
        self.mappub=self.create_publisher(String,'/navigation/mapping_status',10)
        self.stoppub=self.create_publisher(Bool,'/navigation/stop_requested',10)
        self.service=self.create_service(Trigger,'/navigation/frontiers',self.frontier,callback_group=self.group)
        self.planner=ActionServer(self,ComputePathToPose,'/compute_path_to_pose',self.plan,callback_group=self.group,cancel_callback=lambda h:CancelResponse.ACCEPT)
        self.nav=ActionServer(self,NavigateToPose,'/navigate_to_pose',self.drive,callback_group=self.group,cancel_callback=lambda h:CancelResponse.ACCEPT)
        self.timer=self.create_timer(.05,self.publish,callback_group=self.group)
        self.client=ActionClient(self,NavigateToPose,'/navigation/navigate_to_goal',callback_group=self.group)
    def publish(self):
        self.rawpub.publish(self.raw);self.infpub.publish(self.raw)
        self.guardpub.publish(String(data=json.dumps(dict(control_permitted=self.healthy,pose=self.pose,reason_code='' if self.healthy else 'robot_state_timeout'))))
        self.mappub.publish(String(data=json.dumps(dict(ready=True,session='fixture',known_cells=sum(x>=0 for x in self.raw.data),revision=1,reset_generation=self.reset_generation))))
    def frontier(self,req,res):res.success=True;res.message=json.dumps(dict(frontiers=frontiers()));return res
    def plan(self,h):
        p=h.request.goal.pose.position;res=ComputePathToPose.Result();info=self.raw.info
        i=int(np.floor((p.y-info.origin.position.y)/info.resolution));j=int(np.floor((p.x-info.origin.position.x)/info.resolution))
        if 0<=i<info.height and 0<=j<info.width and self.raw.data[i*info.width+j]>=0:res.path.poses=[h.request.goal];h.succeed()
        else:h.abort()
        return res
    def drive(self,h):
        p=h.request.pose.pose.position;self.calls.append([p.x,p.y]);deadline=time.monotonic()+(4 if self.slow else .2)
        while time.monotonic()<deadline:
            if h.is_cancel_requested:h.canceled();return NavigateToPose.Result()
            time.sleep(.03)
        self.pose=[p.x,p.y,0.]
        if self.reveal:self.raw=grid(4.)
        h.succeed();return NavigateToPose.Result()


def wait(future,timeout=8):
    deadline=time.monotonic()+timeout
    while not future.done() and time.monotonic()<deadline:time.sleep(.03)
    assert future.done(),'异步接口超时'
    return future.result()


def submit(fixture):
    goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.pose.orientation.w=1.;goal.pose.pose.position.x=3.5
    h=wait(fixture.client.send_goal_async(goal));assert h.accepted;return h


def test_ros_unknown_to_final_cancel_fault_and_replacement(tmp_path,monkeypatch):
    monkeypatch.setenv('FOREST_TERRAIN_SCENE','fixture');monkeypatch.setenv('FOREST_NAV_MISSION_RESULTS',str(tmp_path))
    monkeypatch.setenv('FOREST_NAV_NO_CANDIDATE_TIMEOUT','0.5')
    import rclpy
    from forest_nav_control.unknown_navigation import UnknownNavigation
    rclpy.init();f=Fixture();node=UnknownNavigation();executor=MultiThreadedExecutor(num_threads=10)
    executor.add_node(f);executor.add_node(node);thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        assert f.client.wait_for_server(timeout_sec=3);time.sleep(.3)
        h=submit(f);res=wait(h.get_result_async(),12)
        assert res.status==4 and len(f.calls)==2
        records=[json.loads(p.read_text()) for p in tmp_path.glob('*.json')]
        assert records[0]['events'][0]['initial_target_unknown'] and any(e['phase']=='exploring' for e in records[0]['events'])
        assert records[0]['known_cells_final']>records[0]['known_cells_initial']
        # 停止整个任务，随后等待，确认不会又提交下一观察点。
        f.raw=grid();f.pose=[0.,0.,0.];f.slow=True;time.sleep(.3);before=len(f.calls)
        h=submit(f);deadline=time.monotonic()+3
        while len(f.calls)==before and time.monotonic()<deadline:time.sleep(.03)
        wait(h.cancel_goal_async());assert wait(h.get_result_async()).status==5
        canceled_calls=len(f.calls);time.sleep(.5);assert len(f.calls)==canceled_calls
        # 替换时先取消旧子目标，再执行新任务；旧目标不能迟到覆盖新目标。
        h1=submit(f);time.sleep(.4);h2=submit(f)
        assert wait(h1.get_result_async()).status==6
        wait(h2.cancel_goal_async());assert wait(h2.get_result_async()).status==5
        # 故障撤销后不会因许可恢复自行续跑。
        h=submit(f);time.sleep(.4);f.healthy=False
        assert wait(h.get_result_async()).status==6
        before=len(f.calls);f.healthy=True;time.sleep(.4);assert len(f.calls)==before
        # 与统一停止按钮相同的通知取消整个任务，不能恢复重新发目标。
        h=submit(f);time.sleep(.4);f.stoppub.publish(Bool(data=True))
        assert wait(h.get_result_async()).status==6
        before=len(f.calls);time.sleep(.4);assert len(f.calls)==before
        # 地图短暂重置又恢复ready时，旧任务仍须因代次改变退出。
        h=submit(f);time.sleep(.4);f.reset_generation+=1
        assert wait(h.get_result_async()).status==6
        assert node.state['reason_code']=='map_reset'
        before=len(f.calls);time.sleep(.2);assert len(f.calls)==before
        # 没有新地图证据时，成功抵达观察点也不能无限兜圈；预算耗尽必须终止。
        f.slow=False;f.reveal=False;f.raw=grid();f.pose=[0.,0.,0.];time.sleep(.3)
        node.observe=lambda handle,token,duration:node.check(handle,token)
        h=submit(f);assert wait(h.get_result_async(),12).status==6
        assert node.state['reason_code']=='no_candidate_available' and node.state['attempts']<=12
    finally:
        with node.lock:node.epoch+=1
        executor.shutdown(timeout_sec=5);thread.join(5);f.destroy_node();node.destroy_node();rclpy.shutdown()


def test_manager_heartbeat_and_cancel_barrier():
    from forest_nav_control.mission_status import manager_fault
    assert manager_fault({},10.,10.2)==('','')
    assert manager_fault({},10.,11.1)[0]=='mission_manager_timeout'
    assert manager_fault({'reason_code':'cancel_confirmation_timeout'},10.,10.2)[0]=='cancel_confirmation_timeout'
    assert manager_fault({'phase':'blocked','reason_code':'recovery_budget_exhausted'},10.,10.2)==('','')


@pytest.mark.parametrize('reveal', [True, False])
def test_ros_outside_goal_expands_or_stops_with_budget(tmp_path,monkeypatch,reveal):
    """实际ROS Action通信验证：图外终点仅用于探索方向，绝不先发给下层导航。

    导航/感知为夹具，覆盖扩图编排和有限失败，不宣称Isaac机器人走到远目标。
    """
    import rclpy
    from forest_nav_control.unknown_navigation import UnknownNavigation
    monkeypatch.setenv('FOREST_TERRAIN_SCENE','fixture')
    monkeypatch.setenv('FOREST_NAV_MISSION_RESULTS',str(tmp_path))
    monkeypatch.setenv('FOREST_NAV_NO_CANDIDATE_TIMEOUT','0.5')
    class ExpandingFixture(Fixture):
        def drive(self,h):
            result=super().drive(h)
            if reveal:
                # 模拟观察点到达后收到新证据：复用生产TerrainGrid整数扩图逻辑。
                from forest_nav_control.terrain_costmap import TerrainGrid
                terrain=TerrainGrid(size=10.,expand=True)
                terrain.ensure_bounds(np.array([[7.,0.]]))
                assert terrain.width>50
                msg=OccupancyGrid();msg.header.frame_id='map';msg.info.resolution=.2
                msg.info.width=msg.info.height=terrain.width
                msg.info.origin.position.x=msg.info.origin.position.y=terrain.origin
                msg.info.origin.orientation.w=1.
                yy,xx=np.indices(terrain.cost.shape)
                x=terrain.origin+(xx+.5)*.2;y=terrain.origin+(yy+.5)*.2
                terrain.cost[(abs(y)<2)&(x>=-2)&(x<7.5)]=0
                msg.data=terrain.cost.ravel().tolist();self.raw=msg
            return result
    rclpy.init();f=ExpandingFixture();f.reveal=False
    node=UnknownNavigation()
    node.observe=lambda handle,token,duration:node.check(handle,token)
    executor=MultiThreadedExecutor(num_threads=10)
    executor.add_node(f);executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        assert f.client.wait_for_server(timeout_sec=3);time.sleep(.3)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
        goal.pose.pose.orientation.w=1.;goal.pose.pose.position.x=6.5
        h=wait(f.client.send_goal_async(goal));assert h.accepted
        result=wait(h.get_result_async(),12)
        record=json.loads(next(tmp_path.glob('*.json')).read_text())
        assert record['initial_target_unknown'] and record['initial_target_outside']
        assert any(e.get('reason_code')=='target_outside_current_map' for e in record['events'])
        assert any(e['phase']=='exploring' for e in record['events'])
        assert f.calls and f.calls[0][0]<2.
        if reveal:
            assert result.status==4 and record['phase']=='arrived'
            assert f.calls[-1]==[6.5,0.]
            assert len(f.calls)>=2
            assert all(call!=[6.5,0.] for call in f.calls[:-1])
            assert not record['target_outside_current_map']
        else:
            assert result.status==6 and record['reason_code']=='no_candidate_available'
            assert record['attempts']<=12 and record['failures']<=6
            assert all(call!=[6.5,0.] for call in f.calls)
        before=len(f.calls);time.sleep(.3);assert len(f.calls)==before
    finally:
        with node.lock:node.epoch+=1
        executor.shutdown(timeout_sec=5);thread.join(5)
        f.destroy_node();node.destroy_node();rclpy.shutdown()


def test_long_running_mission_keeps_health_and_cancel_guards(monkeypatch):
    """正常运行超过旧总时限不结束；健康撤销和人工取消仍须生效。"""
    from types import SimpleNamespace
    from unittest.mock import patch
    from forest_nav_control.unknown_navigation import UnknownNavigation,MissionFault,MissionStopped
    from forest_nav_control.task_client import TaskClient
    import inspect
    now=time.monotonic()
    monkeypatch.setenv('FOREST_NAV_MISSION_TIMEOUT','0.01')
    node=SimpleNamespace(epoch=1,started=now-100000.,guard_received=now,
        guard={'control_permitted':True},mapping_received=now,
        mapping={'ready':True,'session':'test','reset_generation':0},
        session='test',map_generation=0,costmap_received=now)
    handle=SimpleNamespace(is_cancel_requested=False)
    with patch('forest_nav_control.unknown_navigation.rclpy.ok',return_value=True):
        UnknownNavigation.check(node,handle,1)
        node.guard={'control_permitted':False,'reason_code':'robot_state_timeout'}
        with pytest.raises(MissionFault,match='robot_state_timeout'):UnknownNavigation.check(node,handle,1)
        handle.is_cancel_requested=True
        with pytest.raises(MissionStopped):UnknownNavigation.check(node,handle,1)
    assert inspect.signature(TaskClient.wait_task).parameters['timeout'].default==0


def test_ros_final_success_waits_for_parking_and_reports_drift(tmp_path,monkeypatch):
    """上层Action不得将原生Nav2成功直接作为停稳成功；失败和取消均保留。"""
    import rclpy
    from forest_nav_control.unknown_navigation import UnknownNavigation
    monkeypatch.setenv('FOREST_TERRAIN_SCENE','fixture')
    monkeypatch.setenv('FOREST_NAV_DUAL_POLICY','1')
    monkeypatch.setenv('FOREST_NAV_MISSION_RESULTS',str(tmp_path))
    class ParkingFixture(Fixture):
        def __init__(self):
            self.settled=False;self.sim_time=1.;self.offset=0.
            super().__init__()
            self.bridgepub=self.create_publisher(String,'/navigation/bridge_status',20)
        def publish(self):
            super().publish()
            if hasattr(self,'bridgepub'):
                self.sim_time+=.05
                self.bridgepub.publish(String(data=json.dumps(dict(ready=True,sim_time=self.sim_time,
                    locomotion=dict(mode='standing' if self.settled else 'braking',settled=self.settled)))))
        def drive(self,h):
            result=super().drive(h)
            if h.request.pose.pose.position.x==3.5:self.pose[0]+=self.offset
            return result
    def await_settling(node,pending):
        deadline=time.monotonic()+8
        while node.state.get('phase')!='settling' and time.monotonic()<deadline:
            assert not pending.done();time.sleep(.03)
        assert node.state['phase']=='settling'
        time.sleep(.2);assert not pending.done()
    rclpy.init();f=ParkingFixture();node=UnknownNavigation()
    executor=MultiThreadedExecutor(num_threads=10)
    executor.add_node(f);executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    try:
        assert f.client.wait_for_server(timeout_sec=3);time.sleep(.3)
        h=submit(f);pending=h.get_result_async();await_settling(node,pending)
        f.settled=True;assert wait(pending).status==4
        assert node.state['parking_status']=='verified'
        f.settled=False;f.offset=.5;time.sleep(.2)
        h=submit(f);pending=h.get_result_async();await_settling(node,pending)
        f.settled=True;assert wait(pending).status==6
        assert node.state['reason_code']=='parking_pose_out_of_tolerance'
        assert node.state['parking_status']=='failed'
        f.settled=False;f.offset=0.;time.sleep(.2)
        h=submit(f);pending=h.get_result_async();await_settling(node,pending)
        wait(h.cancel_goal_async());assert wait(pending).status==5
    finally:
        with node.lock:node.epoch+=1
        executor.shutdown(timeout_sec=5);thread.join(5)
        f.destroy_node();node.destroy_node();rclpy.shutdown()
