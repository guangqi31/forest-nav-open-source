"""终点持续位姿控制节点；不转发或仲裁Nav2速度。

远处使用 Nav2，原生到达后由 /navigation/terminal_pose 接管。终点 Action
成功仅代表首次稳定到位；节点随后保留同一目标监测并纠偏，通过状态话题
报告保持状态。人工停止、新目标、失联、地图重置会撤销保持，绝不追旧目标。
所有纠偏依据 LIO/里程计；没有使用仿真根位姿或强制移动机器人。
"""
import os
import json
import math
import threading
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Bool, Empty
from std_srvs.srv import Trigger

from .goal_validation import GoalMapValidator
from .terminal_core import TerminalConfig, TerminalController


class TerminalNode(Node):
    def __init__(self):
        super().__init__('terminal_pose_controller', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.group = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        self.config_file = Path(__file__).resolve().parents[1]/'config/terminal_control.json'
        self.cfg = TerminalConfig(**json.loads(self.config_file.read_text()))
        self.pending_cfg = None
        self.validator = GoalMapValidator(self)
        self.data = {}; self.received = {}
        self.native_active = False; self.native_id = None
        self.controller = None; self.current = None; self.goal_id = None
        self.goal_context = None; self.fault = ''; self.last_stamp = None
        self.command = (0., 0., 0.); self.status = {}; self.reserved = False
        self.motion_inhibited = False
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/navigation/terminal_status', 10)
        self.subs = []
        for key, topic in [('bridge','/navigation/bridge_status'), ('guard','/navigation/task_status'), ('mapping','/navigation/mapping_status')]:
            self.subs.append(self.create_subscription(String, topic,
                lambda msg, key=key: self.receive_json(key, msg), 10, callback_group=self.group))
        self.subs.append(self.create_subscription(Odometry, '/odom', self.on_odom, 10, callback_group=self.group))
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.subs.append(self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
            lambda msg: self.receive('local_map', msg), qos, callback_group=self.group))
        self.subs.append(self.create_subscription(GoalStatusArray, ('/follow_path/_action/status' if os.environ.get('FOREST_NAV_PLANNER')=='far' else '/navigate_to_pose/_action/status'), self.on_native, qos, callback_group=self.group))
        self.subs.append(self.create_subscription(Bool, '/navigation/stop_requested', self.on_stop, 10, callback_group=self.group))
        self.subs.append(self.create_subscription(Empty,'/navigation/terminal_release',self.release_hold,10,callback_group=self.group))
        self.release_service = self.create_service(Trigger, '/navigation/terminal_release', self.release, callback_group=self.group)
        self.server = ActionServer(self, NavigateToPose, '/navigation/terminal_pose', self.execute,
            goal_callback=self.accept_goal, cancel_callback=lambda handle: CancelResponse.ACCEPT,
            handle_accepted_callback=self.accepted, callback_group=self.group)
        self.timer = self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME), callback_group=self.group)

    def receive(self, key, data):
        with self.lock:
            self.data[key] = data; self.received[key] = time.monotonic()

    def receive_json(self, key, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict): self.receive(key, data)
        except (ValueError, TypeError): pass

    def on_odom(self, msg):
        # Odometry.twist 按 child_frame_id 定义，当前契约必须是 base_link。
        if msg.child_frame_id != 'base_link': return
        v = msg.twist.twist
        values = (v.linear.x, v.linear.y, v.angular.z)
        if all(math.isfinite(x) for x in values): self.receive('velocity', values)


    def on_native(self, msg):
        if not msg.status_list: return
        latest = max(msg.status_list, key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec))
        key = bytes(latest.goal_info.goal_id.uuid).hex()
        with self.lock:
            if key != self.native_id:
                self.native_id = key
                if latest.status in (1, 2): self.motion_inhibited = False
            self.native_active = latest.status in (1, 2, 3)
            # 新 Nav2 任务只能撤销完成后的保持，不能与正在执行的终点 Action 抢控制。
            if self.native_active and self.controller is not None and self.current is None:
                self.clear('replaced_by_navigation')

    def clear(self, reason='released'):
        # 只结束本节点自己的终点输出；导航期间不周期发布零速度。
        if self.controller is not None and not self.native_active:
            self.pub.publish(Twist())
        self.controller = None; self.fault = reason; self.command = (0., 0., 0.)
        self.goal_context = None; self.last_stamp = None
        # 保留 current 给执行回调结束其 Action；新请求必须等旧请求退出。

    def on_stop(self, msg):
        if msg.data:
            with self.lock:
                self.clear('user_stopped'); self.motion_inhibited = True
                self.pub.publish(Twist())

    def release_hold(self,msg):
        with self.lock:
            if self.current is None:self.clear('released')

    def release(self, request, response):
        with self.lock: self.clear('released')
        response.success = True; response.message = '终点保持已撤销，旧目标不会恢复纠偏。'
        return response

    def health(self, now):
        # 只检查实际计算输入是否存在；不以心跳、倾斜或地图状态触发停车。
        bridge=self.data.get('bridge',{})
        pose=bridge.get('pose')
        if not isinstance(pose,list) or len(pose)!=3 or not all(math.isfinite(v) for v in pose):return 'waiting_pose'
        if 'velocity' not in self.data or 'locomotion' not in bridge:return 'waiting_feedback'
        return ''

    def accept_goal(self, request):
        p = request.pose.pose.position; q = request.pose.pose.orientation
        with self.lock:
            try:
                candidate_cfg = TerminalConfig(**json.loads(self.config_file.read_text()))
            except (OSError,ValueError,TypeError) as exc:
                self.get_logger().error('终点配置无效，保留当前任务：'+str(exc))
                return GoalResponse.REJECT
            health = self.health(time.monotonic())
            valid = (not self.reserved and self.current is None and request.pose.header.frame_id == 'map'
                     and all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w))
                     and abs(q.x)+abs(q.y) < 1e-6 and abs(q.z*q.z+q.w*q.w-1) < 1e-3
                     and not health)
            # 是否交接由上层导航决定，不因交接期间的位移再次按距离拒绝。
            if valid:
                self.reserved = True; self.pending_cfg = candidate_cfg
            else: self.get_logger().warning('终点接管拒绝：'+(health or '目标格式或旧任务未结束'))
            return GoalResponse.ACCEPT if valid else GoalResponse.REJECT

    def accepted(self, handle):
        with self.lock:
            p = handle.request.pose.pose
            # 新任务读取完整、已校验的配置快照；执行/保持过程中不热改增益。
            self.cfg = self.pending_cfg
            self.pending_cfg = None
            self.controller = TerminalController((p.position.x, p.position.y, 2*math.atan2(p.orientation.z, p.orientation.w)), self.cfg)
            self.current = handle; self.goal_id = bytes(handle.goal_id.uuid).hex()
            mapping = self.data.get('mapping',{})
            self.goal_context = (mapping.get('session'), mapping.get('reset_generation', 0))
            self.fault = ''; self.last_stamp = self.data['bridge'].get('sim_time')
            self.command = (0., 0., 0.)
            self.motion_inhibited = False
            self.reserved = False
        handle.execute()

    def execute(self, handle):
        result = NavigateToPose.Result()
        while rclpy.ok():
            with self.lock:
                if handle.is_cancel_requested:
                    self.clear('canceled'); self.current = None; handle.canceled(); return result
                if self.controller is None or self.fault:
                    self.current = None; handle.abort(); return result
                if self.controller.verified:
                    self.current = None; handle.succeed(); return result
                feedback = NavigateToPose.Feedback()
                feedback.distance_remaining = float(self.controller.detail.get('position_error_m', 0.))
                handle.publish_feedback(feedback)
            time.sleep(.05)
        with self.lock:
            self.clear('shutdown'); self.current = None
        handle.abort(); return result

    def tick(self):
        now = time.monotonic()
        with self.lock:
            health = self.health(now)
            command = (0., 0., 0.)
            owned = self.controller is not None or self.current is not None
            if self.controller is not None and not self.fault and not health and not self.native_active:
                try:
                    bridge = self.data['bridge']; mapping = self.data.get('mapping',{})
                    pose = bridge['pose']; velocity = self.data['velocity']
                    self.validator.check(self.controller.target, allow_unobserved=False)
                    stamp = float(bridge['sim_time'])
                    dt = max(0., stamp-(self.last_stamp if self.last_stamp is not None else stamp))
                    self.last_stamp = stamp
                    command = self.controller.step(pose, velocity, bridge['locomotion'], dt)
                except (ValueError, RuntimeError, KeyError, TypeError) as exc:
                    health = str(exc); command = (0., 0., 0.)
            # Nav2直接发布/cmd_vel，本节点不接收、不缓存、不改写导航速度。
            # 只有终点Action/保持拥有目标且Nav2已结束时，才发布自身纠偏输出。
            self.command = command
            if owned and not self.native_active:
                msg = Twist(); msg.linear.x, msg.linear.y, msg.angular.z = command
                self.pub.publish(msg)
            core = self.controller
            self.status = dict(schema_version=1, goal_id=self.goal_id,
                phase='fault' if owned and self.fault else (core.phase if core else 'idle'),
                reason_code=self.fault if owned else '', target=list(core.target) if core else None,
                owner='terminal' if owned else ('nav2' if self.native_active else 'none'),
                command=list(self.command), health=health,
                parking_status='failed' if owned and self.fault else ('verified' if core and core.verified else 'settling'),
                parking_detail=core.detail if core else {},
                ever_verified=bool(core and core.ever_verified), updated_wall_time=time.time())
            self.status_pub.publish(String(data=json.dumps(self.status, ensure_ascii=False, allow_nan=False)))


def main():
    rclpy.init(); node = TerminalNode(); executor = MultiThreadedExecutor(num_threads=4); executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        executor.shutdown(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
