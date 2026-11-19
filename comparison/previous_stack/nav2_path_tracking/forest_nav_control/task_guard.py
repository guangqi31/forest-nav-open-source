"""导航状态报告节点（不干预控制）。

观察原生 NavigateToPose 的状态，不另造一套导航 Action。使用墙上单调时间
检查反馈，仅显示诊断，不因此取消目标或拦截速度。静态地图收到一次后保持有效；新地图
必须重新校验。这里只验证地图契约，通行性与完整机器人轮廓由 Nav2 判断。
"""
import os
import json
import time
import math
import rclpy
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from .mission_status import interpret_phase,manager_fault


class TaskGuard(Node):
    def __init__(self):
        super().__init__('navigation_task_guard')
        self.mapping = {}
        self.exploration = {};self.exploration_received=0.
        self.mapping_received = None
        self.map_generation=None
        self.bridge = None
        self.received = None
        self.map_valid = False
        self.revision = 0
        self.goal_id = None
        self.state = 'waiting'
        self.reason = ''
        self.reason_code = ''
        self.session = None
        self.active = False
        self.cancel_pending = None
        self.plan_count = 0
        self.client_records = {}
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.stop_pub=self.create_publisher(Bool,'/navigation/stop_requested',10)
        self.exploration_sub=self.create_subscription(String,'/navigation/exploration_status',self.on_exploration,10)
        self.mission_pub=self.create_publisher(String,'/navigation/mission_status',10)
        self.status_pub = self.create_publisher(String, '/navigation/task_status', 10)
        self.cancel_client = self.create_client(CancelGoal, ('/follow_path/_action/cancel_goal' if os.environ.get('FOREST_NAV_PLANNER')=='far' else '/navigate_to_pose/_action/cancel_goal'))
        self.cancel_service = self.create_service(Trigger, '/navigation/cancel', self.cancel)
        self.mapping_sub = self.create_subscription(String, '/navigation/mapping_status', self.on_mapping, 10)
        self.bridge_sub = self.create_subscription(String, '/navigation/bridge_status', self.on_bridge, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.map_sub = self.create_subscription(OccupancyGrid, '/traversability_map', self.on_map, latched)
        self.action_sub = self.create_subscription(GoalStatusArray, ('/follow_path/_action/status' if os.environ.get('FOREST_NAV_PLANNER')=='far' else '/navigate_to_pose/_action/status'), self.on_action, latched)
        self.records_sub = self.create_subscription(String, '/navigation/task_records', self.on_records, latched)
        self.plan_sub = self.create_subscription(Path, '/plan', self.on_plan, 10)
        self.timer = self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_exploration(self,msg):
        try:self.exploration=json.loads(msg.data);self.exploration_received=time.monotonic()
        except (ValueError,TypeError):pass

    def on_mapping(self, msg):
        try:
            data=json.loads(msg.data)
            if isinstance(data,dict):
                self.mapping=data;self.mapping_received=time.monotonic()
        except (ValueError,TypeError):pass

    def on_records(self, msg):
        """按客户端保存任务显示数据；不让客户端上报覆盖安全监视结论。"""
        try:
            data = json.loads(msg.data)
            if data.get('schema_version') == 1 and isinstance(data.get('source'), str):
                self.client_records[data['source']] = data
        except (ValueError, TypeError, AttributeError):
            pass

    def on_bridge(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or not isinstance(data.get('ready'), bool):
                raise ValueError('状态格式错误')
            self.bridge, self.received = data, time.monotonic()
        except (ValueError, TypeError):
            self.bridge, self.received = None, None

    def on_map(self, msg):
        info = msg.info
        q = info.origin.orientation
        # 目前只支持 map 中轴对齐的二维代价栅格，拒绝旋转地图和非法数值。
        self.map_valid = bool(msg.header.frame_id == 'map' and info.width > 0 and info.height > 0
            and math.isfinite(info.resolution) and info.resolution > 0
            and len(msg.data) == info.width*info.height and all(-1 <= v <= 100 for v in msg.data)
            and all(math.isfinite(v) for v in (info.origin.position.x, info.origin.position.y))
            and abs(q.x)+abs(q.y)+abs(q.z) < 1e-6 and abs(abs(q.w)-1) < 1e-6)
        self.revision += 1

    def on_plan(self, msg):
        if self.active:
            self.plan_count += 1

    def on_action(self, msg):
        if not msg.status_list:
            return
        # 状态数组可能保留已结束的旧目标；按接收时间戳选取最新任务。
        latest = max(msg.status_list, key=lambda s:(s.goal_info.stamp.sec, s.goal_info.stamp.nanosec))
        goal_id = bytes(latest.goal_info.goal_id.uuid).hex()
        if goal_id != self.goal_id:
            self.goal_id, self.reason, self.plan_count = goal_id, '', 0
            self.reason_code = ''
            self.session = None if self.bridge is None else self.bridge.get('session')
            self.map_generation=self.mapping.get('reset_generation',0)
        self.active = latest.status in (1, 2, 3)
        names = {1:'accepted', 2:'navigating', 3:'canceling', 4:'succeeded', 5:'canceled', 6:'failed'}
        self.state = 'failed' if self.reason else names.get(latest.status, 'unknown')

    def request_cancel(self):
        # 零 UUID/时间戳是 ROS Action 的“取消全部当前目标”。这里只用于统一取消
        # 服务；诊断异常不会调用此方法。
        if self.cancel_pending is not None and not self.cancel_pending.done():
            return False
        if not self.cancel_client.service_is_ready():
            return False
        self.cancel_pending = self.cancel_client.call_async(CancelGoal.Request())
        return True

    def cancel(self, request, response):
        # 同时通知上层终止整个探索任务，避免原生子目标取消后又被管理器重新提交。
        self.stop_pub.publish(Bool(data=True))
        response.success = self.request_cancel() or os.environ.get('FOREST_NAV_PLANNER')=='far'
        response.message = '已提交取消请求；最终结果请查看任务状态' if response.success else '取消服务尚未就绪或已有取消请求'
        return response

    def tf_fresh(self):
        try:
            transform = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
            stamp = transform.header.stamp.sec+transform.header.stamp.nanosec/1e9
            # 与反馈中的仿真时间比较，不能拿系统时间减去仿真时间。
            age = self.bridge['sim_time']-stamp
            return -.1 <= age < .5
        except Exception:
            return False

    def tick(self):
        now = time.monotonic()
        reason = ''
        reason_code = ''
        if self.bridge is None or self.received is None or now-self.received > .5 or not self.bridge.get('ready'):
            reason = '实际状态失效或反馈超时'
            reason_code = 'robot_state_timeout'
        elif os.environ.get('FOREST_NAV_MAP_MODE') in ('online','lidar') and (self.mapping_received is None or now-self.mapping_received>2. or not self.mapping.get('ready')):
            reason = self.mapping.get('reason') or '等待在线点云地图更新'
            reason_code = 'mapping_not_ready'
        elif os.environ.get('FOREST_NAV_MAP_MODE')=='lidar' and self.mapping.get('source')!=('cmu_lio' if os.environ.get('FOREST_NAV_LOCALIZATION')=='lio' else 'cmu_lidar'):
            reason = '等待CMU激光地形地图，拒绝其他地图来源'
            reason_code = 'wrong_map_source'
        elif os.environ.get('FOREST_NAV_MAP_MODE')=='lidar' and manager_fault(self.exploration,self.exploration_received,now)[0]:
            reason_code,reason=manager_fault(self.exploration,self.exploration_received,now)
        elif not self.map_valid:
            reason = '缺少有效的 map 代价栅格'
            reason_code = 'map_invalid'
        elif not self.tf.can_transform('map', 'base_link', rclpy.time.Time()):
            reason = '缺少 map 到 base_link 的定位变换'
            reason_code = 'tf_missing'
        elif self.bridge.get('sim_time') is not None and not self.tf_fresh():
            reason = '定位变换停止更新'
            reason_code = 'tf_stale'
        elif self.active and self.map_generation is not None and self.map_generation!=self.mapping.get('reset_generation',0):
            reason='地图已重置，旧任务必须结束'
            reason_code='map_reset'
        elif self.active and self.session != self.bridge.get('session'):
            reason = '仿真实例已变化，旧目标停止执行'
            reason_code = 'session_changed'
        # 用户要求撤销额外的监视拦截。诊断异常只显示，不修改动作结果，
        # 不调用request_cancel，也不发布任何控制许可；人工取消服务单独保留。
        if self.goal_id is None:
            self.state = 'idle'
        status = dict(state=self.state, reason='', reason_code='', goal_id=self.goal_id,
                      monitoring_only=True, diagnostic_reason=reason, diagnostic_code=reason_code,
                      map_valid=self.map_valid, tf_available=self.tf.can_transform('map', 'base_link', rclpy.time.Time()), map_revision=self.revision,
                      client_records=self.client_records,
                      plan_count=self.plan_count, pose=None if self.bridge is None else self.bridge.get('pose'))
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        records=[record for client in self.client_records.values() for record in client.get('recent_tasks',[])]
        requests=[client.get('current_task') for client in self.client_records.values() if client.get('current_task')]
        request=max(requests,key=lambda record:record.get('created_wall_time',0)) if requests else None
        active_record=next((record for record in records if record.get('goal_id')==self.goal_id),None)
        phase=interpret_phase(self.state,self.active,True,bool(self.mapping.get('ready',self.map_valid)),
                              request,False)
        mission=dict(schema_version=1,phase=phase,
            reason=self.reason or reason or ((request or {}).get('reason','') if not self.active else ''),
            reason_code=self.reason_code or reason_code or ((request or {}).get('reason_code','') if not self.active else ''),
            task_id=(active_record or {}).get('task_id'),goal_id=self.goal_id,
            current_request=request,monitoring_only=True,diagnostic_reason=reason,diagnostic_code=reason_code,pose=status['pose'],
            map_revision=self.mapping.get('revision',self.revision),mapping=self.mapping,
            parking_status=(active_record or {}).get('parking_status','unverified'),
            exploration_enabled=False)
        if os.environ.get('FOREST_NAV_MAP_MODE')=='lidar':
            mission['exploration_enabled']=True
            if now-self.exploration_received<1. and self.exploration.get('goal_id'):
                # 上层phase解释整个任务；本节点只汇总显示，不干预执行。
                for key in ('phase','reason_code','target','subgoal','goal_id','final_leg','attempts','failures','parking_status','parking_detail'):
                    if key in self.exploration:mission[key]=self.exploration[key]
                mission['exploration']=self.exploration
        self.mission_pub.publish(String(data=json.dumps(mission,ensure_ascii=False)))


def main():
    rclpy.init()
    node = TaskGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
