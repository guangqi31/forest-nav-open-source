"""FAR→Nav2 FollowPath任务适配；不实现前沿评分、黑名单或全局规划。

上层Action保留用户原始map目标及朝向；FAR图在连续lio_odom中运行。
任务ID贯穿FAR请求与路线。人工取消和替换通过标准Action取消请求处理，不设置永久故障锁。
"""
import copy
import json
import math
import threading
import time
import uuid
from pathlib import Path
import rclpy
from rclpy.action import ActionServer,ActionClient,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock,ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav2_msgs.action import NavigateToPose,FollowPath
from nav_msgs.msg import Path as NavPath,OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String,Bool,Empty
from visibility_graph_msg.msg import ForestGoal,ForestRoute
from .goal_validation import GoalMapValidator
from .far_path import sample_route,route_length
from .terminal_core import TerminalConfig
import os


class Interrupted(RuntimeError):pass


class FarNavigation(Node):
    def __init__(self):
        super().__init__('forest_far_navigation',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.group=ReentrantCallbackGroup();self.lock=threading.RLock();self.serial=threading.Lock()
        self.epoch=0;self.tokens={};self.outstanding=0
        self.bridge={};self.mapping={};self.received={};self.route=None;self.route_version=0
        self.follow_handles=[];self.pending_sends=[];self.terminal_handle=None;self.task_context=None
        self.current=None;self.task_id=None;self.last_route_wall=0.;self.events=[];self.state=dict(phase='ready',planner='far')
        self.cfg=TerminalConfig(**json.loads((Path(__file__).resolve().parents[1]/'config/terminal_control.json').read_text()))
        self.validator=GoalMapValidator(self)
        self.follow=ActionClient(self,FollowPath,'/follow_path',callback_group=self.group)
        self.terminal=ActionClient(self,NavigateToPose,'/navigation/terminal_pose',callback_group=self.group)
        self.release_pub=self.create_publisher(Empty,'/navigation/terminal_release',10)
        self.use_terminal=os.environ.get('FOREST_NAV_TERMINAL_CONTROL','1')=='1'
        self.request_pub=self.create_publisher(ForestGoal,'/far/request',10)
        self.reset_pub=self.create_publisher(Empty,'/far/reset_graph',10)
        self.plan_pub=self.create_publisher(NavPath,'/plan',10)
        self.status_pub=self.create_publisher(String,'/navigation/exploration_status',10)
        self.ready_pub=self.create_publisher(String,'/navigation/far_status',10)
        self.subs=[self.create_subscription(ForestRoute,'/far/route',self.on_route,10,callback_group=self.group),
            self.create_subscription(String,'/navigation/bridge_status',lambda m:self.receive('bridge',m),10,callback_group=self.group),
            self.create_subscription(String,'/navigation/mapping_status',lambda m:self.receive('mapping',m),10,callback_group=self.group),
            self.create_subscription(Bool,'/navigation/stop_requested',self.stop,10,callback_group=self.group)]
        self.server=ActionServer(self,NavigateToPose,'/navigation/navigate_to_goal',self.execute,
            goal_callback=self.accept,cancel_callback=lambda h:CancelResponse.ACCEPT,
            handle_accepted_callback=self.accepted,callback_group=self.group)
        self.timer=self.create_timer(.1,self.publish,clock=Clock(clock_type=ClockType.STEADY_TIME),callback_group=self.group)
        self.control_active=False
        self.local_grid=None
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.local_sub=self.create_subscription(OccupancyGrid,'/local_costmap/costmap',self.on_local,qos,callback_group=self.group)

    def on_local(self,msg):
        if msg.header.frame_id=='odom':self.local_grid=msg

    def receive(self,key,msg):
        try:
            data=json.loads(msg.data)
            if isinstance(data,dict):
                with self.lock:
                    old=self.mapping if key=='mapping' else None
                    if old and (old.get('session'),old.get('reset_generation',0))!=(data.get('session'),data.get('reset_generation',0)):
                        self.reset_pub.publish(Empty())
                    setattr(self,key,data);self.received[key]=time.monotonic()
        except (ValueError,TypeError):pass

    def on_route(self,msg):
        with self.lock:
            self.last_route_wall=time.monotonic()
            self.far_ready=msg.ready
            if msg.task_id==self.task_id:
                self.route=msg;self.route_version+=1

    def stop(self,msg):
        if msg.data:
            with self.lock:self.epoch+=1;self.control_active=False

    def health(self):
        # 只报告初始化所需输入；不以状态年龄或ready报告否决执行。
        if 'pose' not in self.bridge:return 'waiting_pose'
        if not getattr(self,'far_ready',False):return 'far_initializing'
        return ''

    def publish(self):
        with self.lock:
            self.state.update(schema_version=1,planner='far',exploration_enabled=True,updated_wall_time=time.time())
            self.status_pub.publish(String(data=json.dumps(self.state,ensure_ascii=False)))
            self.ready_pub.publish(String(data=json.dumps(dict(ready=not self.health(),
                reason=self.health(),planner='far'))))

    def transition(self,phase,reason='',**fields):
        with self.lock:
            self.state.update(phase=phase,reason_code=reason,**fields)
            self.events.append(dict(wall_time=time.time(),phase=phase,reason_code=reason,**fields))

    def accept(self,request):
        p=request.pose.pose.position;q=request.pose.pose.orientation
        valid=(request.pose.header.frame_id=='map' and all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w))
            and abs(q.x)+abs(q.y)<1e-6 and abs(q.z*q.z+q.w*q.w-1)<1e-3)
        with self.lock:
            if not valid:return GoalResponse.REJECT
            self.outstanding+=1
        return GoalResponse.ACCEPT

    def accepted(self,handle):
        with self.lock:
            self.epoch+=1;self.tokens[bytes(handle.goal_id.uuid)]=self.epoch
        handle.execute()

    def check(self,handle,token):
        if not rclpy.ok() or handle.is_cancel_requested or token!=self.epoch:raise Interrupted('canceled_or_replaced')
    def wait(self,future,handle=None,token=None):
        while not future.done():
            if handle is not None:self.check(handle,token)
            if not rclpy.ok():raise Interrupted('shutdown')
            time.sleep(.02)
        return future.result()

    def track_send(self,client,goal):
        future=client.send_goal_async(goal)
        # Store pending acceptance before waiting; cleanup includes late responses.
        self.pending_sends.append(future)
        return future

    def stop_actions(self):
        self.control_active=False
        def cancel_accepted(f):
            try:
                h=f.result()
                if h.accepted and h.status not in (4,5,6):h.cancel_goal_async()
            except Exception as exc:self.get_logger().warning('取消请求失败：'+str(exc))
        for f in self.pending_sends:f.add_done_callback(cancel_accepted)
        handles=list(self.follow_handles)
        if self.terminal_handle is not None:handles.append(self.terminal_handle)
        for h in handles:
            if h.status not in (4,5,6):h.cancel_goal_async()
        self.pending_sends=[];self.follow_handles=[];self.terminal_handle=None
        self.release_pub.publish(Empty())

    def finish_following(self, handle, token):
        """接管前确认FollowPath真正结束，避免Nav2取消时的零指令覆盖纠偏。"""
        pending=list(self.pending_sends)
        followers=list(self.follow_handles)
        self.stop_actions()
        for future in pending:
            follower=self.wait(future,handle,token)
            if follower.accepted and follower not in followers:
                followers.append(follower)
        for follower in followers:
            if follower.status not in (4,5,6):
                follower.cancel_goal_async()
            self.wait(follower.get_result_async(),handle,token)

    def far_request(self,*,cancel=False,target=None):
        req=ForestGoal();req.header.frame_id='lio_odom';req.header.stamp=self.get_clock().now().to_msg()
        req.task_id=self.task_id;req.cancel=cancel
        if target is not None:
            x,y,angle=self.mapping.get('map_to_odom',(0.,0.,0.));c,s=math.cos(angle),math.sin(angle)
            dx,dy=target[0]-x,target[1]-y
            req.goal.x=c*dx+s*dy;req.goal.y=-s*dx+c*dy
            req.goal.z=0.  # FAR associates the goal height from measured terrain.
        self.request_pub.publish(req)

    def path_message(self,points):
        path=NavPath();path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
        for i,p in enumerate(points):
            out=PoseStamped();out.header=path.header;out.pose.position.x=float(p[0]);out.pose.position.y=float(p[1])
            other=points[min(i+1,len(points)-1)]
            yaw=math.atan2(other[1]-p[1],other[0]-p[0]) if i<len(points)-1 else (
                math.atan2(p[1]-points[i-1][1],p[0]-points[i-1][0]) if i else 0.)
            out.pose.orientation.z=math.sin(yaw/2);out.pose.orientation.w=math.cos(yaw/2);path.poses.append(out)
        return path

    def executable(self,route):
        if route.header.frame_id!='lio_odom' or route.path.header.frame_id!='lio_odom':return []
        # 保留FAR完整折线，仅做坐标转换与加密，不查地图、不裁断路线。
        x,y,a=self.mapping.get('map_to_odom',(0.,0.,0.));c,s=math.cos(a),math.sin(a)
        points=[(p.pose.position.x,p.pose.position.y) for p in route.path.poses]
        points=sample_route(points)
        return [(c*p[0]-s*p[1]+x,s*p[0]+c*p[1]+y) for p in points]

    def execute(self,handle):
        key=bytes(handle.goal_id.uuid);token=self.tokens[key];result=NavigateToPose.Result()
        with self.serial:
            if token!=self.epoch:
                handle.abort()
                with self.lock:self.outstanding-=1;self.tokens.pop(key,None)
                return result
            self.current=handle;self.task_id=uuid.uuid4().hex;self.route=None;self.route_version=0;self.events=[]
            self.task_context=(self.mapping.get('session'),self.mapping.get('reset_generation',0))
            p=handle.request.pose.pose;target=(p.position.x,p.position.y,2*math.atan2(p.orientation.z,p.orientation.w))
            self.state=dict(phase='navigating',goal_id=key.hex(),target=list(target),reason_code='',parking_status='unverified')
            try:
                self.check(handle,token)
                self.stop_actions()
                self.validator.check(target,allow_unobserved=True)
                self.far_request(target=target)
                last_version=-1;last_send=0.;last_request=time.monotonic();active_result=None
                while True:
                    self.check(handle,token)
                    pose=self.bridge.get('pose')
                    if pose is None:time.sleep(.02);continue
                    if self.use_terminal and math.dist(pose[:2],target[:2])<=self.cfg.handoff_radius_m:
                        self.validator.check(target,allow_unobserved=False)
                        self.finish_following(handle,token)
                        self.far_request(cancel=True)
                        self.transition('settling',parking_status='settling')
                        goal=NavigateToPose.Goal();goal.pose=copy.deepcopy(handle.request.pose)
                        sent=self.track_send(self.terminal,goal);h=self.wait(sent,handle,token)
                        if not h.accepted:
                            time.sleep(.02);continue
                        self.terminal_handle=h;done=h.get_result_async()
                        while not done.done():self.check(handle,token);time.sleep(.02)
                        if done.result().status!=4:raise RuntimeError('terminal_control_failed')
                        self.terminal_handle=None;self.pending_sends=[]
                        self.transition('arrived',parking_status='verified');handle.succeed();break
                    route=self.route
                    if route is None or route.task_id!=self.task_id:
                        if time.monotonic()-last_request>1.:
                            self.far_request(target=target);last_request=time.monotonic()
                        time.sleep(.02);continue
                    if active_result is not None and active_result.done():
                        status=active_result.result().status;self.control_active=False;active_result=None
                        if not self.use_terminal and status==4 and math.dist(pose[:2],target[:2])<=self.cfg.position_tolerance_m:
                            self.transition('arrived',parking_status='unverified');handle.succeed();break
                        self.plan_pub.publish(self.path_message([]))
                        self.transition('waiting_route','follow_path_status_'+str(status),
                            route_points=0,route_length_m=0.,last_follow_path_status=status)
                        # 消费过的版本保持已消费，等FAR下一次更新；不立即重发失败旧路径。
                    if self.route_version!=last_version and time.monotonic()-last_send>=.2:
                        last_version=self.route_version
                        points=self.executable(route) if route.valid and not route.canceled else []
                        if len(points)>=2:
                            goal=FollowPath.Goal();goal.path=self.path_message(points)
                            if not self.use_terminal and math.dist(points[-1],target[:2])<=self.cfg.position_tolerance_m:
                                goal.path.poses[-1].pose.orientation=copy.deepcopy(handle.request.pose.pose.orientation)
                            goal.controller_id='TransitPath' if self.use_terminal else 'FollowPath';goal.goal_checker_id='transit_goal_checker' if self.use_terminal else 'goal_checker'
                            sent=self.track_send(self.follow,goal);h=self.wait(sent,handle,token)
                            if not h.accepted:
                                self.plan_pub.publish(self.path_message([]))
                                self.transition('waiting_route','follow_path_rejected',
                                    route_points=0,route_length_m=0.,far_route_valid=bool(route.valid))
                                time.sleep(.02);continue
                            self.follow_handles.append(h);active_result=h.get_result_async()
                            # Retain in-flight results until terminal state; do not cancel them on update.
                            self.follow_handles=[v for v in self.follow_handles if v.status not in (4,5,6)]
                            self.pending_sends=[v for v in self.pending_sends if not v.done()]
                            self.control_active=True;self.plan_pub.publish(goal.path)
                            self.transition('navigating',route_points=len(points),route_length_m=route_length(points),
                                far_route_valid=True,route_version=last_version)
                            last_send=time.monotonic()
                        else:
                            # 仅清理失效路线的显示和统计，不取消旧Action、不插入零速度。
                            self.plan_pub.publish(self.path_message([]))
                            self.transition('waiting_observation','no_executable_far_path',
                                route_points=0,route_length_m=0.,far_route_valid=False,
                                route_version=last_version)
                    feedback=NavigateToPose.Feedback();feedback.distance_remaining=math.dist(pose[:2],target[:2])
                    handle.publish_feedback(feedback);time.sleep(.02)
            except Interrupted:
                try:self.far_request(cancel=True);self.stop_actions()
                except RuntimeError as exc:self.transition('fault',str(exc))
                else:self.transition('canceled','canceled_or_replaced')
                if handle.is_cancel_requested:handle.canceled()
                else:handle.abort()
            except Exception as exc:
                try:self.far_request(cancel=True);self.stop_actions()
                except RuntimeError as barrier:exc=barrier
                self.transition('fault',str(exc));handle.abort()
            finally:
                folder=Path(os.environ.get('FOREST_NAV_MISSION_RESULTS', str(Path(__file__).resolve().parents[2]/'forest_far_navigation/results/missions')))
                try:
                    folder.mkdir(parents=True,exist_ok=True)
                    (folder/(key.hex()+'.json')).write_text(json.dumps(dict(self.state,events=self.events),ensure_ascii=False,indent=2))
                except OSError as exc:self.get_logger().error('FAR任务记录失败：'+str(exc))
                self.current=None;self.control_active=False
                self.plan_pub.publish(self.path_message([]))
                self.outstanding-=1;self.tokens.pop(key,None)
        return result


def main():
    rclpy.init();node=FarNavigation();executor=MultiThreadedExecutor(num_threads=6);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        node.epoch+=1
        try:node.far_request(cancel=True);node.stop_actions()
        except Exception:pass
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
