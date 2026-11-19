"""ROS硬件通信桥：实际反馈→时钟/物理安全；适配后速度→仿真。

定时器采用单调时钟，仿真暂停时也能发现失联。发布的里程计来自实际反馈，
不能用指令冒充。LIO模式只消费估计定位，不发布真值odom/TF；显式truth模式
才保留原来的水平机体真值里程计。
"""
import json
import math
import os
import socket
import time
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMessage
from action_msgs.msg import GoalStatusArray
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import String, Bool
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from .sim_endpoint import STATE_PORT, COMMAND_PORT, finite_vector, path_packet


class SimBridge(Node):
    def __init__(self):
        super().__init__('g1_state_bridge')
        self.use_lio=os.environ.get('FOREST_NAV_LOCALIZATION')=='lio'
        self.localization={};self.localization_received=None
        if self.use_lio:
            self.localization_sub=self.create_subscription(String,'/navigation/localization_status',self._localization,10)
        self.declare_parameter('state_port', STATE_PORT)
        self.declare_parameter('command_port', COMMAND_PORT)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('127.0.0.1', self.get_parameter('state_port').value))
        self.socket.setblocking(False)
        self.destination = ('127.0.0.1', self.get_parameter('command_port').value)
        self.state, self.state_received = None, None
        self.command, self.command_received = (0., 0., 0.), None
        self.plan_points = []
        self.plan_revision = 0
        self.plan_dirty = False
        self.plan_last_sent = 0.0
        self.path_send_error = ''
        self.sequence = 0
        self.clock_pub = self.create_publisher(ClockMessage, '/clock', 10)
        self.odom_pub = None if self.use_lio else self.create_publisher(Odometry, '/odom', 10)
        self.status_pub = self.create_publisher(String, '/navigation/bridge_status', 10)
        self.command_sub = self.create_subscription(Twist, '/cmd_vel_adapted', self._command, 10)
        self.plan_sub = self.create_subscription(PathMessage, '/navigation/world_plan' if self.use_lio else '/plan', self._plan, 1)
        self.action_sub = self.create_subscription(
            GoalStatusArray, ('/follow_path/_action/status' if os.environ.get('FOREST_NAV_PLANNER')=='far' else '/navigate_to_pose/_action/status'), self._action, 1)
        self.tf_pub = None if self.use_lio else TransformBroadcaster(self)
        if not self.use_lio:
            self.static_pub = StaticTransformBroadcaster(self)
            identity = TransformStamped()
            identity.header.frame_id, identity.child_frame_id = 'map', 'odom'
            identity.transform.rotation.w = 1.
            self.static_pub.sendTransform(identity)
        self.timer = self.create_timer(.02, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('桥接节点已启动：实际状态 → odom/TF/clock，cmd_vel_adapted → 仿真')

    def _localization(self,msg):
        try:
            data=json.loads(msg.data)
            if data.get('source')!='fast_lio' or not isinstance(data.get('ready'),bool):return
            if data.get('pose') is not None and not finite_vector(data['pose'],3):return
            self.localization=data;self.localization_received=time.monotonic()
        except (ValueError,TypeError):pass

    def _command(self, msg):
        values = [msg.linear.x, msg.linear.y, msg.angular.z]
        if not finite_vector(values, 3):
            self.command, self.command_received = (0.,0.,0.), None
            return
        self.command, self.command_received = tuple(values), time.monotonic()

    def _plan(self, msg):
        if msg.header.frame_id != ('isaac_world' if self.use_lio else 'map'):
            return
        points = [[float(pose.pose.position.x), float(pose.pose.position.y)]
                  for pose in msg.poses]
        if len(points) > 4096 or not all(
                math.isfinite(value) for point in points for value in point):
            return
        self.plan_points = points
        self.plan_revision += 1
        self.plan_dirty = True

    def _action(self, msg):
        if not msg.status_list:
            return
        latest = max(msg.status_list,
                     key=lambda status: (status.goal_info.stamp.sec,
                                         status.goal_info.stamp.nanosec))
        if latest.status not in (1, 2, 3) and self.plan_points:
            self.plan_points = []
            self.plan_revision += 1
            self.plan_dirty = True

    def _accept(self, packet):
        now = time.monotonic()
        if not isinstance(packet,dict):
            return False
        if (packet.get('version') != 1 or not isinstance(packet.get('session'), str) or
            not isinstance(packet.get('sequence'), int) or not finite_vector(packet.get('pose'),3) or
            not finite_vector(packet.get('velocity'),3) or not isinstance(packet.get('ready'),bool)):
            return False
        age = (time.monotonic_ns() - int(packet['sent_ns'])) / 1e9
        sim_time = float(packet['sim_time'])
        if not (0 <= age < .5) or not math.isfinite(sim_time) or sim_time <= 0:
            return False
        if self.state is not None:
            if packet['session'] != self.state['session']:
                # 不在运动过程中无声切换仿真实例；先等待上一实例超时。
                if now - self.state_received < .5:
                    return False
                self.command, self.command_received = (0.,0.,0.), None
            elif (packet['sequence'] <= self.state['sequence'] or sim_time <= self.state['sim_time']):
                return False
        self.state, self.state_received = packet, now
        return True

    def _publish_state(self):
        state = self.state
        stamp = ClockMessage()
        total_ns = round(state['sim_time'] * 1e9)
        stamp.clock.sec, stamp.clock.nanosec = divmod(total_ns, 1000000000)
        self.clock_pub.publish(stamp)
        # LIO模式的仿真反馈只推进时钟/硬件安全；严禁继续发布真值odom或定位TF。
        if self.use_lio:return
        x,y,yaw = state['pose']
        odom = Odometry()
        odom.header.stamp = stamp.clock
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x,y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = state['velocity']
        self.odom_pub.publish(odom)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x, transform.transform.translation.y = x,y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_pub.sendTransform(transform)

    def _tick(self):
        updated = False
        for _ in range(100):
            try:
                data,_ = self.socket.recvfrom(8192)
            except BlockingIOError:
                break
            try:
                updated = self._accept(json.loads(data)) or updated
            except (ValueError,TypeError,KeyError,OverflowError):
                continue
        if updated:
            self._publish_state()
        now = time.monotonic()
        age = None if self.state_received is None else now-self.state_received
        ready = bool(self.state)
        hardware_ready=ready
        # 硬件心跳不延长定位寿命；分别检查状态心跳和原始LIO测量时刻。
        localization_age = None if self.localization_received is None else now-self.localization_received
        localization_stamp = self.localization.get('stamp')
        localization_ready = bool(not self.use_lio or (
            self.localization.get('ready') is True and localization_age is not None
            and 0 <= localization_age < 1.
            and isinstance(localization_stamp, (int, float)) and math.isfinite(localization_stamp)
            and self.state and -.1 <= self.state['sim_time']-localization_stamp < .5))
        command_fresh = self.command_received is not None
        command = self.command
        if self.state is not None:
            self.sequence += 1
            packet = {'version':1, 'session':self.state['session'], 'sequence':self.sequence,
                      'sent_ns':time.monotonic_ns(), 'command':command}
            # 速度始终先发小包；路线显示独立发送，失败不影响速度和看门狗。
            self.socket.sendto(json.dumps(packet,allow_nan=False).encode(),self.destination)
            if self.plan_dirty or (self.plan_points and now-self.plan_last_sent > .5):
                try:
                    encoded = path_packet(self.plan_points, session=self.state['session'], revision=self.plan_revision)
                    self.socket.sendto(encoded, self.destination)
                except (OSError, ValueError) as exc:
                    self.path_send_error = str(exc)
                else:
                    self.path_send_error = ''
                    self.plan_dirty = False
                self.plan_last_sent = now
        status = String()
        status.data = json.dumps({'ready':ready,'state_age_sec':age,'command_fresh':command_fresh,'localization_ready':localization_ready,
            'session':None if self.state is None else self.state['session'],
            'pose':self.localization.get('pose') if self.use_lio else (None if self.state is None else self.state['pose']),
            'localization_age_sec':localization_age,'localization_stamp':localization_stamp,
            'path_send_error':getattr(self,'path_send_error',''),
            'localization_source':'fast_lio' if self.use_lio else 'isaac_ground_truth','hardware_ready':hardware_ready,
            'sim_time':None if self.state is None else self.state['sim_time'],
            'fault':None if self.state is None else self.state.get('fault',''),
            'locomotion':None if not ready or self.state is None else self.state.get('locomotion')})
        self.status_pub.publish(status)

    def destroy_node(self):
        self.socket.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = SimBridge()
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
