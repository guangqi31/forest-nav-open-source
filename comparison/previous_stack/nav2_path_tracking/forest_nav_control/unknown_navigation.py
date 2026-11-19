"""未知环境目标管理节点：最终坐标 → 已知安全观察点 → 原生Nav2 → G1。

上层使用同一种 NavigateToPose Action，但独立入口为 /navigation/navigate_to_goal。
原生 /navigate_to_pose 只接收已知且经路径验证的子目标。管理器不发布速度、
不训练策略、不清除障碍。线程中的等待只等待异步ROS结果，其他回调继续运行。
"""
import json
import math
import os
from pathlib import Path
import threading
import time
import rclpy
from rclpy.action import ActionClient,ActionServer,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock,ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose,ComputePathToPose
from nav_msgs.msg import OccupancyGrid,Path as NavigationPath
from std_msgs.msg import String,Bool
from std_srvs.srv import Trigger
from .goal_validation import GoalMapValidator,GoalRejected
from .exploration_policy import observation_candidates
from .dual_policy import parking_result
from .terminal_core import TerminalConfig


class MissionStopped(RuntimeError):pass
class MissionFault(RuntimeError):pass


class UnknownNavigation(Node):
    def __init__(self):
        super().__init__('forest_unknown_navigation',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.group=ReentrantCallbackGroup()
        self.nav=ActionClient(self,NavigateToPose,'/navigate_to_pose',callback_group=self.group)
        self.plan=ActionClient(self,ComputePathToPose,'/compute_path_to_pose',callback_group=self.group)
        self.frontiers=self.create_client(Trigger,'/navigation/frontiers',callback_group=self.group)
        self.validator=GoalMapValidator(self)
        self.inflated=None;self.costmap_received=0.;self.guard={};self.guard_received=0.
        self.mapping={};self.mapping_received=0.
        self.lock=threading.RLock();self.serial=threading.Lock();self.epoch=0;self.tokens={};self.outstanding=0
        self.current=None;self.current_epoch=0;self.native=None;self.cancel_barrier_fault=False
        self.last_path_failure='';self.last_drive_failure=''
        self.last_target_state={}
        self.plan_revision=0;self.latest_plan_goal=None
        self.require_parking=os.environ.get('FOREST_NAV_DUAL_POLICY','0')=='1'
        self.use_terminal=os.environ.get('FOREST_NAV_TERMINAL_CONTROL')=='1'
        self.terminal=ActionClient(self,NavigateToPose,'/navigation/terminal_pose',callback_group=self.group) if self.use_terminal else None
        self.terminal_release=self.create_client(Trigger,'/navigation/terminal_release',callback_group=self.group) if self.use_terminal else None
        self.terminal_native=None
        self.terminal_status={};self.terminal_received=0.
        self.terminal_config=TerminalConfig(**json.loads((Path(__file__).resolve().parents[1]/'config/terminal_control.json').read_text())) if self.use_terminal else None
        if self.use_terminal:
            self.terminal_sub=self.create_subscription(String,'/navigation/terminal_status',self.on_terminal,10,callback_group=self.group)
        self.bridge={};self.bridge_received=0.
        self.state=dict(schema_version=1,phase='ready',reason_code='',exploration_enabled=True)
        self.events=[];self.started=0.;self.pose=None;self.session=None;self.map_generation=None
        self.pub=self.create_publisher(String,'/navigation/exploration_status',10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # 使用Nav2现有选择器区分途经点与最终终点；保留消息供行为树首次订阅读取。
        self.controller_selector=self.create_publisher(String,'/navigation/controller_selector',qos)
        self.goal_checker_selector=self.create_publisher(String,'/navigation/goal_checker_selector',qos)
        self.subs=[self.create_subscription(OccupancyGrid,'/global_costmap/costmap',self.costmap,qos,callback_group=self.group),
            self.create_subscription(String,'/navigation/task_status',self.on_guard,10,callback_group=self.group),
            self.create_subscription(String,'/navigation/mapping_status',self.on_mapping,10,callback_group=self.group),
            self.create_subscription(String,'/navigation/bridge_status',self.on_bridge,20,callback_group=self.group),
            self.create_subscription(Bool,'/navigation/stop_requested',self.on_stop,10,callback_group=self.group),
            self.create_subscription(NavigationPath,'/plan',self.on_plan,10,callback_group=self.group)]
        self.server=ActionServer(self,NavigateToPose,'/navigation/navigate_to_goal',self.execute,
            goal_callback=self.goal_callback,cancel_callback=lambda handle:CancelResponse.ACCEPT,
            handle_accepted_callback=self.accepted,callback_group=self.group)
        self.timer=self.create_timer(.2,self.publish,clock=Clock(clock_type=ClockType.STEADY_TIME),callback_group=self.group)

    def on_bridge(self,msg):
        try:
            self.bridge=json.loads(msg.data);self.bridge_received=time.monotonic()
        except (ValueError,TypeError):pass

    def wait_for_parking(self,target,handle,token):
        """原生Nav2到达后继续保留上层Action，直到新鲜反馈确认站稳且位置达标。

        没有任务总倒计时；人工取消、地图代次或健康变化仍由check处理。
        必须看到本次到达后的新反馈，不能复用上一任务的settled标记。
        """
        if self.use_terminal:
            return self.control_terminal(target,handle,token)
        if not self.require_parking:
            self.state['parking_status']='unverified'
            return
        initial_stamp=self.bridge.get('sim_time',0.)
        self.transition('settling',parking_status='settling')
        while True:
            self.check(handle,token)
            data=self.bridge;locomotion=data.get('locomotion')
            if time.monotonic()-self.bridge_received>.5 or not isinstance(locomotion,dict):
                raise MissionFault('locomotion_feedback_timeout')
            if not data.get('ready'):raise MissionFault('robot_state_timeout')
            if data.get('sim_time',0.)>initial_stamp and self.pose:
                parking,detail=parking_result(target,self.pose,locomotion)
                self.state.update(parking_status=parking,parking_detail=detail)
                if parking=='verified':return
                if parking=='failed':raise MissionFault('parking_pose_out_of_tolerance')
            time.sleep(.05)

    def on_terminal(self,msg):
        """Action完成后仍显示保持/再纠偏；历史成功结果不被改写。"""
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):return
        self.terminal_status=data;self.terminal_received=time.monotonic()
        if data.get('goal_id')!=self.state.get('terminal_goal_id') or not self.state.get('terminal_goal_id'):return
        if data.get('phase')=='idle':return
        self.state.update(terminal_phase=data.get('phase'),parking_status=data.get('parking_status'),
                          parking_detail=data.get('parking_detail',{}),hold_reason=data.get('reason_code',''))
        if self.current is None and self.state.get('phase') in ('arrived','holding','hold_fault'):
            self.state['phase']='hold_fault' if data.get('phase')=='fault' else ('arrived' if data.get('phase')=='holding' else 'holding')

    def control_terminal(self,target,handle,token):
        """原生跟踪结束后获取独占速度控制权；不设置整任务倒计时。"""
        self.cancel_native()
        if not self.terminal.server_is_ready():raise MissionFault('terminal_not_ready')
        request=NavigateToPose.Goal();request.pose=self.pose_message(target)
        sent=self.terminal.send_goal_async(request)
        sent.add_done_callback(lambda f:self.cancel_late(f,token))
        nh=self.await_future(sent,handle,token)
        if not nh.accepted:raise MissionFault('terminal_goal_rejected')
        self.terminal_native=nh
        self.state['terminal_goal_id']=bytes(nh.goal_id.uuid).hex()
        self.transition('settling',parking_status='settling')
        future=nh.get_result_async()
        try:
            while not future.done():
                self.check(handle,token)
                time.sleep(.05)
            if future.result().status!=GoalStatus.STATUS_SUCCEEDED:raise MissionFault('terminal_control_failed')
            self.state['parking_status']='verified'
        finally:
            if not future.done():nh.cancel_goal_async()
            self.terminal_native=None

    def release_terminal(self):
        """新任务或失败撤销旧保持；停止按钮另由终点节点直接监听，避免取消竞态。"""
        if self.terminal_native is not None:self.terminal_native.cancel_goal_async()
        if self.terminal_release is not None and self.terminal_release.service_is_ready():
            return self.terminal_release.call_async(Trigger.Request())
        return None

    def costmap(self,msg):
        if msg.header.frame_id=='map' and len(msg.data)==msg.info.width*msg.info.height:
            self.inflated=msg;self.costmap_received=time.monotonic()

    def on_guard(self,msg):
        try:
            data=json.loads(msg.data);self.guard=data;self.guard_received=time.monotonic();self.pose=data.get('pose')
        except (ValueError,TypeError):pass

    def on_mapping(self,msg):
        try:self.mapping=json.loads(msg.data);self.mapping_received=time.monotonic()
        except (ValueError,TypeError):pass

    def on_stop(self,msg):
        if msg.data:
            with self.lock:self.epoch+=1
            if self.current is None and self.state.get('phase') in ('arrived','holding','hold_fault'):
                self.transition('canceled','hold_released_by_user')

    def goal_callback(self,request):
        p=request.pose.pose.position;q=request.pose.pose.orientation
        valid=request.pose.header.frame_id=='map' and all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w))
        valid=valid and abs(q.x)+abs(q.y)<1e-6 and abs(q.z*q.z+q.w*q.w-1)<1e-3
        # 最多一条执行中、一条等待替换，避免快速点击耗尽执行器线程。
        with self.lock:
            if not valid or self.outstanding>=2 or self.cancel_barrier_fault:return GoalResponse.REJECT
            self.outstanding+=1
        return GoalResponse.ACCEPT

    def accepted(self,handle):
        with self.lock:
            self.epoch+=1;self.tokens[bytes(handle.goal_id.uuid)]=self.epoch
        handle.execute()

    def publish(self):
        self.pub.publish(String(data=json.dumps(dict(self.state,updated_wall_time=time.time()),ensure_ascii=False)))

    def transition(self,phase,code='',**fields):
        self.state.update(phase=phase,reason_code=code,**fields)
        event=dict(elapsed_sec=round(time.monotonic()-self.started,3),phase=phase,reason_code=code,**fields)
        self.events.append(event);self.get_logger().info(json.dumps(event,ensure_ascii=False))
        self.publish()

    def check(self,handle,token):
        if not rclpy.ok() or handle.is_cancel_requested or token!=self.epoch:raise MissionStopped('canceled_or_replaced')
        now=time.monotonic()
        # 移除项目额外的状态许可/地图心跳取消：诊断不改变正在执行的目标。
        # 手动取消、目标替换、坐标会话变化仍单独处理；原生Nav2负责自身执行。
        if self.session!=self.mapping.get('session'):raise MissionFault('session_changed')
        if self.map_generation!=self.mapping.get('reset_generation',0):raise MissionFault('map_reset')

    def await_future(self,future,handle,token,timeout=5.):
        deadline=time.monotonic()+timeout
        while not future.done():
            self.check(handle,token)
            if time.monotonic()>deadline:raise MissionFault('service_timeout')
            time.sleep(.03)
        return future.result()

    def pose_message(self,target):
        msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.position.x,msg.pose.position.y=map(float,target[:2]);yaw=target[2]
        msg.pose.orientation.z=math.sin(yaw/2);msg.pose.orientation.w=math.cos(yaw/2)
        return msg

    def has_path(self,target,handle,token):
        """只调用现有Smac规划，不在管理器中另写寻路算法。"""
        self.last_path_failure=''
        request=ComputePathToPose.Goal();request.goal=self.pose_message(target);request.planner_id='GridBased';request.use_start=False
        sent=self.plan.send_goal_async(request)
        # 即使任务中途撤销，迟到的规划响应也不能遗留后台任务。
        sent.add_done_callback(lambda f:self.cancel_late(f,token))
        ph=self.await_future(sent,handle,token)
        if not ph.accepted:
            self.last_path_failure='planner_goal_rejected';return False
        try:
            response=self.await_future(ph.get_result_async(),handle,token,5.)
            if response.status!=GoalStatus.STATUS_SUCCEEDED:
                self.last_path_failure=f'planner_status_{response.status}';return False
            if not response.result.path.poses:
                self.last_path_failure='empty_path';return False
            return True
        except Exception:
            self.last_path_failure='path_check_exception'
            ph.cancel_goal_async();raise

    def target_state(self,target):
        """检查候选在当前原始图和膨胀图中的格状态。"""
        state=dict(valid=False,raw_cost=None,inflated_cost=None,reason='')
        try:
            assessment=self.validator.check(target,allow_unobserved=False)
            state.update(valid=True,raw_cost=None if assessment is None else assessment.get('cost'))
        except GoalRejected as exc:
            state.update(reason=exc.reason,cost=getattr(exc,'cost',None));return state
        if self.inflated is None or time.monotonic()-self.costmap_received>2.:
            state.update(valid=False,reason='costmap_not_ready');return state
        info=self.inflated.info
        col=math.floor((float(target[0])-info.origin.position.x)/info.resolution)
        row=math.floor((float(target[1])-info.origin.position.y)/info.resolution)
        if not (0<=col<info.width and 0<=row<info.height):
            state.update(valid=False,reason='outside_inflated');return state
        cost=int(self.inflated.data[row*info.width+col])
        state['inflated_cost']=cost
        if cost<0:
            state.update(valid=False,reason='unknown_inflated');return state
        if cost>=99:
            state.update(valid=False,reason='occupied_inflated');return state
        return state

    def cancel_late(self,future,token):
        try:
            h=future.result()
            if h.accepted and (token!=self.epoch or not rclpy.ok()):h.cancel_goal_async()
        except Exception:pass

    def cancel_native(self):
        """取消必须得到结果确认后才允许替换目标，避免旧控制器和新目标并行。"""
        if self.native is None:return
        handle,result=self.native
        if not result.done():
            handle.cancel_goal_async();deadline=time.monotonic()+5.
            while rclpy.ok() and not result.done() and time.monotonic()<deadline:time.sleep(.03)
            if not result.done():
                # 无法确认取消时拒绝新的上层目标；控制器独立看门狗继续兜底。
                self.cancel_barrier_fault=True
                raise MissionFault('cancel_confirmation_timeout')
        self.native=None

    def on_plan(self,msg):
        """记录原生规划结果的终点，仅用于同步模式切换，不拦截导航速度。"""
        if msg.header.frame_id!='map' or not msg.poses:return
        p=msg.poses[-1].pose
        self.latest_plan_goal=(p.position.x,p.position.y,2*math.atan2(p.orientation.z,p.orientation.w))
        self.plan_revision+=1

    def select_tracking_mode(self,final):
        """中间点不要求终点朝向；最终目标仍使用原控制器和到达判据。

        始终运行同一棵行为树，由Nav2原生选择器切换插件，因此更新目标
        可以直接抢占旧目标，无须先取消、等待停车再另起一次导航。
        """
        self.controller_selector.publish(String(data='FollowPath' if final else 'TransitPath'))
        self.goal_checker_selector.publish(String(data='goal_checker' if final else 'transit_goal_checker'))

    def drive(self,target,final,handle,token):
        self.check(handle,token)
        self.last_drive_failure=''
        self.last_target_state={}
        plan_revision=self.plan_revision
        selected_final=False
        # 沿用已有终点局部控制的1米范围来提前准备下一段；短段最多提前
        # 半程，确保机器人实际经过本段，而不是在起点连续消耗观察点。
        initial_distance=math.dist(self.pose[:2],target[:2]) if self.pose else 0.
        transit_radius=min(self.terminal_config.capture_radius_m if self.use_terminal else 1.,initial_distance/2)
        if not final:self.select_tracking_mode(False)
        request=NavigateToPose.Goal();request.pose=self.pose_message(target)
        # 不取消仍在行进的上一观察点；同一行为树接受新目标后原生抢占。
        sent=self.nav.send_goal_async(request)
        sent.add_done_callback(lambda f:self.cancel_late(f,token))
        nh=self.await_future(sent,handle,token)
        if not nh.accepted:
            self.last_target_state=self.target_state(target)
            self.last_drive_failure=('target_invalidated' if not self.last_target_state['valid']
                                     else 'native_goal_rejected');return False
        result=nh.get_result_async();self.native=(nh,result)
        # 此处再次检查取消，覆盖“发送请求后、收到接受响应前按停止”的竞态。
        self.check(handle,token)
        self.transition('navigating' if final else 'exploring',subgoal=list(target),subgoal_id=bytes(nh.goal_id.uuid).hex(),final_leg=final)
        deadline=time.monotonic()+float(os.environ.get('FOREST_NAV_SUBGOAL_TIMEOUT','70'))
        while not result.done():
            self.check(handle,token)
            # 新目标接受与新路径生成是两个异步事件。只有看到终点一致的
            # 新路径后才启用最终对齐，避免把旧观察点yaw交给最终控制器。
            planned=self.latest_plan_goal
            # Smac2D终点位于目标栅格中心；允许该格半对角线的量化误差，
            # 不把真实目标坐标改成栅格中心，也不改变最终停车验收容差。
            plan_tolerance=self.inflated.info.resolution/math.sqrt(2)+1e-5 if self.inflated else 1e-5
            if (final and not selected_final and self.plan_revision>plan_revision and planned
                    and math.dist(planned[:2],target[:2])<=plan_tolerance
                    and abs(math.atan2(math.sin(planned[2]-target[2]),math.cos(planned[2]-target[2])))<1e-5):
                self.select_tracking_mode(True);selected_final=True
            if not final:
                # 中间点只是展开地图的观察位置，不必坚持走到一个已被Nav2
                # 膨胀图标为占用的点。立即让外层选择新点，旧Action保留运行，
                # 下一目标通过原生抢占接管；这里不取消、不发零速度。
                # 仅响应已有占用判据，不因软代价变大、地图暂未就绪或
                # 停稳代价阈值而新增运行中的停车/换点条件。
                target_state = self.target_state(target)
                if target_state.get('reason') in ('occupied', 'occupied_inflated'):
                    self.last_drive_failure = 'target_invalidated'
                    self.last_target_state = target_state
                    self.transition('exploring', 'subgoal_refresh_required',
                                    subgoal=list(target), final_leg=False,
                                    target_state=target_state)
                    return False
            # 观察点只用于展开下一段规划，不是停车位。提前准备期间保留
            # self.native及运行中的Action，不发布零速度、不请求取消。
            if not final and self.pose and math.dist(self.pose[:2],target[:2])<=transit_radius:
                self.transition('exploring','subgoal_passed',subgoal=list(target),final_leg=False)
                return True
            if (final and self.use_terminal and self.pose
                    and math.dist(self.pose[:2],target[:2])<=self.terminal_config.handoff_radius_m
                    and time.monotonic()-self.terminal_received<.5
                    and not self.terminal_status.get('health') and self.terminal.server_is_ready()):
                # 接管不等于到达。先确认原生Action取消完成，再允许终点控制发速度。
                self.cancel_native()
                self.transition('settling','terminal_handoff',parking_status='settling')
                return True
            if time.monotonic()>deadline:
                self.last_drive_failure='subgoal_timeout'
                self.cancel_native();return False
            feedback=NavigateToPose.Feedback()
            if self.pose:
                feedback.current_pose=self.pose_message(self.pose)
                feedback.distance_remaining=math.hypot(self.pose[0]-self.state['target'][0],self.pose[1]-self.state['target'][1])
            feedback.navigation_time.sec=int(time.monotonic()-self.started)
            handle.publish_feedback(feedback)
            time.sleep(.1)
        self.check(handle,token)
        status=result.result().status;self.native=None
        # 原生Action被外部取消时，上层也结束，不能继续重新发送目标。
        if status==GoalStatus.STATUS_CANCELED:
            raise MissionStopped('native_goal_canceled')
        if status!=GoalStatus.STATUS_SUCCEEDED:
            self.last_target_state=(self.target_state(target) if not final else {})
            self.last_drive_failure=('target_invalidated'
                if self.last_target_state and not self.last_target_state['valid']
                else f'action_status_{status}')
            return False
        return True

    def execute(self,handle):
        key=bytes(handle.goal_id.uuid);token=self.tokens[key]
        result=NavigateToPose.Result()
        # 替换目标排队到上一任务完全取消之后。检查token可以淘汰已过时的等待任务。
        with self.serial:
            self.started=time.monotonic();self.current=handle;self.current_epoch=token;self.events=[]
            self.session=self.mapping.get('session');self.map_generation=self.mapping.get('reset_generation',0);p=handle.request.pose.pose
            target=(p.position.x,p.position.y,2*math.atan2(p.orientation.z,p.orientation.w))
            self.state=dict(schema_version=1,phase='ready',reason_code='',exploration_enabled=True,
                goal_id=key.hex(),target=list(target),session=self.session,subgoal=None)
            excluded=[];failures=0;attempts=0;invalidations=0;no_candidate_since=None
            try:
                self.check(handle,token)
                if self.use_terminal:
                    if not self.terminal_release.service_is_ready():raise MissionFault('terminal_not_ready')
                    released=self.await_future(self.release_terminal(),handle,token)
                    if not released.success:raise MissionFault('terminal_release_failed')
                initial=self.validator.check(target,allow_unobserved=True) or {}
                self.transition('ready',initial_target_unknown=initial.get('unknown',False),
                    initial_target_outside=initial.get('outside',False),known_cells_initial=self.mapping.get('known_cells'))
                while True:
                    self.check(handle,token)
                    # 整次任务最多12段、6次失败；无前沿时只短暂等待新观测，不盲目倒退。
                    if attempts>=12 or failures>=6:raise MissionFault('recovery_budget_exhausted')
                    # 每次观测后重查终点：范围外→图内未知→已知安全，才可发最终目标。
                    # 禁行/高代价/无效图仍立即结束，不能被“允许探索”绕过。
                    try:assessment=self.validator.check(target,allow_unobserved=True) or {}
                    except GoalRejected as exc:
                        self.state['goal_rejection']=dict(reason=exc.reason,message=str(exc),
                            cost=getattr(exc,'cost',None),
                            max_stopping_cost=getattr(exc,'max_stopping_cost',None))
                        raise MissionFault(exc.reason)
                    final_known=not assessment.get('unknown',False)
                    self.state['target_outside_current_map']=assessment.get('outside',False)
                    if final_known and self.has_path(target,handle,token):
                        attempts+=1
                        if self.drive(target,True,handle,token):
                            self.wait_for_parking(target,handle,token)
                            self.transition('arrived',known_cells_final=self.mapping.get('known_cells'));handle.succeed();break
                        failures+=1
                        self.transition('waiting_observation','final_leg_failed',
                                        drive_failure=self.last_drive_failure)
                        self.observe(handle,token,1.)
                        continue
                    path_failure=(self.last_path_failure
                                  if final_known else '')
                    self.transition('waiting_observation',
                        ('target_outside_current_map' if assessment.get('outside') else 'target_unknown')
                        if not final_known else 'final_path_unavailable',
                        path_failure=path_failure)
                    if not self.frontiers.service_is_ready():raise MissionFault('frontier_service_unavailable')
                    response=self.await_future(self.frontiers.call_async(Trigger.Request()),handle,token)
                    if not response.success:raise MissionFault('frontier_search_failed')
                    frontier_payload=json.loads(response.message)['frontiers']
                    candidate_details=observation_candidates(
                        self.validator.grid,self.inflated,frontier_payload,
                        self.pose,target,excluded,with_metadata=True)
                    candidates=[entry['point'] for entry in candidate_details]
                    path_checks=[]
                    invalidated_candidates=0
                    selected=None
                    for candidate,detail in zip(candidates,candidate_details):
                        target_state_before=self.target_state(candidate)
                        path_checks.append(dict(
                            candidate=list(candidate),target_state=target_state_before,
                            ok=False,failure='target_invalidated'))
                        if not target_state_before['valid']:
                            excluded.append(candidate)
                            invalidated_candidates+=1
                            continue
                        ok=self.has_path(candidate,handle,token)
                        path_checks[-1].update(ok=ok,failure=self.last_path_failure)
                        if ok:
                            selected=candidate
                            selected_detail=detail
                            break
                        target_state_after=self.target_state(candidate)
                        path_checks[-1]['target_state_after']=target_state_after
                        if not target_state_after['valid']:
                            invalidated_candidates+=1
                        excluded.append(candidate)
                    decision=dict(frontier_count=len(frontier_payload),
                        candidate_count=len(candidates),
                        invalidated_candidates=invalidated_candidates,
                        candidates=candidate_details,
                        path_checks=path_checks,
                        selected=selected)
                    if selected is None:
                        if candidates and invalidated_candidates<len(candidates):
                            failures+=1
                            no_candidate_since=None
                            self.transition('waiting_observation','no_reachable_frontier',
                                failures=failures,decision=decision)
                            self.observe(handle,token,2.)
                            continue
                        # 无候选或候选全部已被新地图判为占用，只等待新观测，
                        # 不把每一轮等待都算作硬失败。
                        if no_candidate_since is None:
                            no_candidate_since=time.monotonic()
                        waited=time.monotonic()-no_candidate_since
                        no_candidate_timeout=float(
                            os.environ.get('FOREST_NAV_NO_CANDIDATE_TIMEOUT','20'))
                        if waited>no_candidate_timeout:
                            raise MissionFault('no_candidate_available')
                        self.transition('waiting_observation','no_candidate_available',
                            failures=failures,invalidations=invalidations,
                            no_candidate_wait_sec=waited,decision=decision)
                        self.observe(handle,token,1.5)
                        continue
                    no_candidate_since=None
                    attempts+=1;ok=self.drive(selected,False,handle,token)
                    excluded.append(selected)
                    invalidated=bool(not ok and self.last_drive_failure=='target_invalidated')
                    if invalidated:
                        invalidations+=1
                        transitions='subgoal_invalidated'
                    elif not ok:
                        failures+=1
                        transitions='subgoal_failed'
                    else:
                        transitions='continue_after_subgoal'
                    if not ok:
                        self.transition('waiting_observation',transitions,
                            excluded=[list(x) for x in excluded],failures=failures,
                            invalidations=invalidations,
                            decision=decision,selected_detail=selected_detail,
                            drive_failure=self.last_drive_failure,
                            target_state=self.last_target_state)
                    else:self.transition('exploring',transitions,
                        excluded=[list(x) for x in excluded],failures=failures,
                        invalidations=invalidations,decision=decision,
                        selected_detail=selected_detail)
                    # 有下一观察点可选时，成功或失败都立即重选并交接。
                    # 无候选分支已有等待新地图的处理，不在此额外固定停车1秒。
            except MissionStopped as exc:
                self.release_terminal()
                self.transition('canceling',str(exc))
                try:self.cancel_native()
                except MissionFault as fault:self.transition('fault',str(fault))
                if handle.is_cancel_requested:handle.canceled()
                else:handle.abort()
                if self.state['phase']!='fault':self.transition('canceled',str(exc))
            except Exception as exc:
                self.release_terminal()
                self.state['fault_context']=dict(mapping=self.mapping,guard={k:v for k,v in self.guard.items() if k!='client_records'},mapping_wall_age=time.monotonic()-self.mapping_received,guard_wall_age=time.monotonic()-self.guard_received)
                try:self.cancel_native()
                except MissionFault as fault:exc=fault
                reason=getattr(exc,'reason',str(exc))
                self.transition('fault' if reason in ('mapping_not_ready','session_changed','control_not_permitted','robot_state_timeout','tf_stale','global_costmap_timeout','cancel_confirmation_timeout','mission_manager_timeout','map_invalid','tf_missing','wrong_map_source','map_reset','locomotion_feedback_timeout') else 'blocked',reason,
                    goal_rejection=self.state.get('goal_rejection'))
                handle.abort()
            finally:
                self.state['attempts']=attempts;self.state['failures']=failures
                self.state['invalidations']=invalidations
                try:self.save_record(key.hex())
                except OSError as exc:self.get_logger().error('任务记录保存失败：'+str(exc))
                self.current=None
                # 已结束任务的迟到服务响应一律撤销，不能在任务失败后又启动运动。
                with self.lock:
                    if self.epoch==token:self.epoch+=1
        with self.lock:self.outstanding-=1;self.tokens.pop(key,None)
        return result

    def target_unknown(self,target):
        try:return bool((self.validator.check(target,allow_unobserved=True) or {}).get('unknown'))
        except GoalRejected:return False

    def observe(self,handle,token,duration):
        deadline=time.monotonic()+duration
        while time.monotonic()<deadline:self.check(handle,token);time.sleep(.05)

    def save_record(self,goal_id):
        directory=Path(os.environ.get('FOREST_NAV_MISSION_RESULTS',str(Path(__file__).resolve().parents[2]/'forest_lidar_perception/results/missions')))
        directory.mkdir(parents=True,exist_ok=True)
        record=dict(self.state,events=self.events,map_revision=self.mapping.get('revision'))
        (directory/(goal_id+'.json')).write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')


def main():
    rclpy.init();node=UnknownNavigation();executor=MultiThreadedExecutor(num_threads=6);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        with node.lock:node.epoch+=1
        executor.shutdown(timeout_sec=6.);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
