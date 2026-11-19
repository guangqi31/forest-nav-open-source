"""中文坐标控制面板：用户输入目标 → NavigateToPose → 已有 G1 策略。

面板是 ROS 的普通客户端，不承担规划或修改策略。Tk 和 ROS 回调在同一线程
轮流执行，所有请求都异步发送；等待仿真加载、路径规划和任务结果时窗口仍可操作。
输入是 map 坐标的米和最终朝向的度，发送 Action 前才把度转换为弧度。
"""
import json
import os
import math
import time
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .goal_validation import GoalMapValidator,GoalRejected
from .task_reporter import TaskReporter
from .mission_status import PHASE_NAMES
from .goal_preview import evaluate_preview


class CoordinateController(Node):
    """控制面板的 ROS 接口，可脱离窗口单独用于集成验收。"""
    def __init__(self):
        super().__init__('g1_coordinate_controller', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.goal_validator = GoalMapValidator(self)
        self.records = TaskReporter(self, self.get_name())
        self.task_tokens = {}
        self.navigator = ActionClient(self, NavigateToPose, '/navigation/navigate_to_goal' if os.environ.get('FOREST_NAV_MAP_MODE')=='lidar' else '/navigate_to_pose')
        self.lifecycle = self.create_client(GetState, ('/controller_server/get_state' if os.environ.get('FOREST_NAV_PLANNER')=='far' else '/bt_navigator/get_state'))
        self.far_status={};self.far_received=0.
        if os.environ.get('FOREST_NAV_PLANNER')=='far':
            self.far_sub=self.create_subscription(String,'/navigation/far_status',self.on_far,10)
        self.cancel_service = self.create_client(Trigger, '/navigation/cancel')
        # 候选目标独立于导航 Action，编辑输入框只更新显示，不触发机器人运动。
        self.preview_publisher = self.create_publisher(String, '/navigation/goal_preview', 1)
        self.pose = None
        self.velocity = None
        self.mission = {}
        self.mission_received = None
        self.bridge = {}
        self.task = {}
        self.bridge_received = self.task_received = None
        self.lifecycle_future = None
        self.lifecycle_checked = None
        self.lifecycle_active = False
        self.last_lifecycle_poll = 0.
        self.pending_goal = None
        self.pending_since = None
        self.goal_handle = None
        self.goal_handle_task_id = None
        self.generation = 0
        self.event = '正在等待仿真和导航服务启动。'
        self.distance_remaining = None
        self.last_result = None
        self.closing = False
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.bridge_sub = self.create_subscription(String, '/navigation/bridge_status', self.on_bridge, 10)
        self.mission_sub=self.create_subscription(String,'/navigation/mission_status',self.on_mission,10)
        self.task_sub = self.create_subscription(String, '/navigation/task_status', self.on_task, 10)

    def on_mission(self,msg):
        try:
            data=json.loads(msg.data)
            if data.get('schema_version')==1 and data.get('phase') in PHASE_NAMES:
                self.mission=data;self.mission_received=time.monotonic()
        except (ValueError,TypeError,AttributeError):pass

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, 2*math.atan2(p.orientation.z, p.orientation.w))
        self.velocity = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_bridge(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError('状态必须是对象')
            self.bridge, self.bridge_received = data, time.monotonic()
        except (ValueError, TypeError):
            self.bridge, self.bridge_received = {}, None

    def on_task(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError('任务状态必须是对象')
            self.task, self.task_received = data, time.monotonic()
        except (ValueError, TypeError):
            self.task, self.task_received = {}, None

    def on_far(self,msg):
        try:self.far_status=json.loads(msg.data);self.far_received=time.monotonic()
        except (ValueError,TypeError):pass

    @property
    def ready(self):
        return bool(self.goal_validator.ready and self.pose is not None
            and self.navigator.server_is_ready())

    def poll(self):
        # 一次最多处理有限数量回调，避免高频里程计占满 Tk 的界面线程。
        for _ in range(8):
            rclpy.spin_once(self, timeout_sec=0.)
        now = time.monotonic()
        if now-self.last_lifecycle_poll > 1. and (self.lifecycle_future is None or self.lifecycle_future.done()):
            self.last_lifecycle_poll = now
            if self.lifecycle.service_is_ready():
                self.lifecycle_future = self.lifecycle.call_async(GetState.Request())
                self.lifecycle_future.add_done_callback(self.on_lifecycle)
            else:
                self.lifecycle_active = False
        if self.pending_since is not None and now-self.pending_since > 5.:
            # 超时仅使这次提交失效。若迟到的响应最终接受目标，on_goal 会取消该目标。
            self.generation += 1
            self.pending_goal, self.pending_since = None, None
            self.event = '目标提交响应超时，已撤销本次提交。请检查导航服务后重试。'
            if self.records.current:
                self.records.update(self.records.current_id, state='failed', reason=self.event)

    def on_lifecycle(self, future):
        try:
            self.lifecycle_active = future.result().current_state.id == 3
            self.lifecycle_checked = time.monotonic()
        except Exception:
            self.lifecycle_active = False

    def send_target(self, x, y, yaw_degrees):
        """提交或替换目标；只在当前反馈和导航服务健康时允许发送。"""
        values = tuple(float(value) for value in (x, y, yaw_degrees))
        if not all(math.isfinite(value) for value in values):
            raise ValueError('目标坐标和朝向必须是有限数字。')
        if self.pending_goal is not None:
            raise RuntimeError('正在提交上一条目标，请稍等。')
        x, y, yaw_degrees = values
        yaw = math.radians(yaw_degrees)
        task_id = self.records.begin((x, y, yaw))
        try:
            if not self.ready:
                raise RuntimeError('仿真或导航尚未就绪，请等待“可以发送目标”。')
            self.goal_validator.check((x,y),allow_unobserved=os.environ.get('FOREST_NAV_MAP_MODE')=='lidar')
        except RuntimeError as exc:
            self.records.update(task_id, state='rejected', reason=str(exc),reason_code=getattr(exc,'reason','not_ready'))
            raise
        request = NavigateToPose.Goal()
        request.pose = PoseStamped()
        request.pose.header.frame_id = 'map'
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x, request.pose.pose.position.y = x, y
        request.pose.pose.orientation.z, request.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.generation += 1
        token = self.generation
        self.task_tokens[token] = task_id
        self.distance_remaining = None
        self.last_result = None
        self.event = f'正在提交目标：X={x:.2f}，Y={y:.2f}，朝向={yaw_degrees:.1f}°。'
        try:
            self.pending_goal = self.navigator.send_goal_async(request,
                feedback_callback=lambda msg: self.on_feedback(msg, token))
        except Exception as exc:
            self.records.update(task_id, state='failed', reason=str(exc))
            raise RuntimeError(f'目标发送失败：{exc}') from exc
        self.pending_since = time.monotonic()
        self.pending_goal.add_done_callback(lambda future: self.on_goal(future, token))

    def on_goal(self, future, token):
        try:
            handle = future.result()
            task_id = self.task_tokens[token]
            if handle.accepted:
                self.records.accepted(task_id, bytes(handle.goal_id.uuid).hex())
                handle.get_result_async().add_done_callback(lambda result: self.on_result(result, token))
            else:
                self.records.update(task_id, state='rejected', reason='Nav2拒绝目标')
            # 用户可能在提交尚未完成时按停止，或已发出更晚的目标。
            # 迟到的旧目标一旦被接受，就取消它自己的 handle，不能取消更新的任务。
            if token != self.generation or self.closing:
                if handle.accepted:
                    self.records.update(task_id, state='canceling', reason='提交已失效，取消迟到的目标')
                    handle.cancel_goal_async()
                return
            self.pending_goal, self.pending_since = None, None
            if not handle.accepted:
                self.event = '导航服务拒绝了目标，请查看导航状态。'
                return
            self.goal_handle = handle
            self.goal_handle_task_id = task_id
            self.event = '目标已接收，正在规划并控制仿真机器人运动。'
        except Exception as exc:
            self.records.update(self.task_tokens[token], state='failed', reason=f'提交目标失败：{exc}')
            if token == self.generation:
                self.pending_goal, self.pending_since = None, None
                self.event = f'提交目标失败：{exc}'

    def on_feedback(self, msg, token):
        if token == self.generation:
            self.distance_remaining = msg.feedback.distance_remaining
        self.records.update(self.task_tokens[token], state='navigating',
            distance_remaining_m=float(msg.feedback.distance_remaining), pose=self.pose)

    def on_result(self, future, token):
        task_id = self.task_tokens[token]
        try:
            response = future.result()
            result = response.result
            self.records.update(task_id, pose=self.pose)
            self.records.result(task_id, response.status, getattr(result, 'error_code', None),
                                getattr(result, 'error_msg', ''))
            if token != self.generation:
                return  # 旧结果写入历史，但不覆盖当前界面与当前目标句柄。
            self.last_result = response.status
            self.distance_remaining = None
            self.goal_handle = None
            self.event = {4:('导航已到达且停稳检查通过。' if self.records.records[task_id].get('parking_status')=='verified' else 'Nav2已到达；机器人停稳尚未验证。'),
                          5:'导航已取消，机器人正在停止。',
                          6:'导航失败：目标不可达或控制执行失败，请检查状态并调整目标。'}.get(
                              self.last_result, '导航任务已结束。')
        except Exception as exc:
            self.records.update(task_id, state='failed', reason=f'读取任务结果失败：{exc}')
            if token == self.generation:
                self.event = f'读取任务结果失败：{exc}'

    def stop(self):
        """取消当前任务；不直接与 Nav2 竞争发布速度指令。"""
        self.generation += 1
        self.pending_goal, self.pending_since = None, None
        self.distance_remaining = None
        if self.records.current and self.records.current['state'] in ('submitting', 'accepted', 'navigating'):
            self.records.update(self.records.current_id, state='canceling', reason='用户请求取消，等待Nav2确认')
        if self.goal_handle is not None:
            self.records.update(self.goal_handle_task_id, state='canceling', reason='用户请求取消，等待Nav2确认')
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        if self.cancel_service.service_is_ready():
            future = self.cancel_service.call_async(Trigger.Request())
            future.add_done_callback(self.on_cancel)
            self.event = '已发送停止请求，等待导航取消及机器人制动。'
        else:
            self.event = '取消服务不可用，请检查导航服务；反馈失联保护仍由桥接端执行。'

    def on_cancel(self, future):
        try:
            if not future.result().success:
                self.event = future.result().message
        except Exception as exc:
            self.event = f'取消请求失败：{exc}'


class ControllerPanel:
    """只负责输入与显示。没有训练按钮，也不需要用户操作多个终端。"""
    STATE_NAMES = {'waiting':'等待就绪', 'idle':'等待目标', 'accepted':'目标已接收',
        'navigating':'正在前往目标', 'canceling':'正在停止', 'succeeded':'Nav2已到达（停稳未验证）',
        'canceled':'已取消（停稳未验证）', 'failed':'导航失败',
        'submitting':'正在提交', 'rejected':'目标被拒绝'}

    def __init__(self, root, node):
        self.root, self.node = root, node
        self.closed = False
        root.title('G1 坐标导航控制器')
        root.geometry('580x720')
        root.minsize(550, 700)
        style = ttk.Style(root)
        style.configure('.', font=('Noto Sans CJK SC', 11))
        style.configure('Title.TLabel', font=('Noto Sans CJK SC', 17, 'bold'))
        style.configure('Go.TButton', font=('Noto Sans CJK SC', 12, 'bold'), padding=9)
        frame = ttk.Frame(root, padding=22)
        frame.pack(fill='both', expand=True)
        ttk.Label(frame, text='输入坐标，让机器人自己走过去', style='Title.TLabel').pack(anchor='w')
        ttk.Label(frame, text='仿真与 G1 策略会自动启动，准备好后点击“前往目标”。', wraplength=475).pack(anchor='w', pady=(6, 14))
        self.connection = tk.StringVar(value='正在启动仿真，请稍候……')
        self.connection_label = ttk.Label(frame, textvariable=self.connection, foreground='#a45e00')
        self.connection_label.pack(anchor='w')
        self.position = tk.StringVar(value='当前位置：等待机器人反馈')
        ttk.Label(frame, textvariable=self.position).pack(anchor='w', pady=(8, 14))
        box = ttk.LabelFrame(frame, text='目标位置（地图坐标）', padding=12)
        box.pack(fill='x')
        # 新林地map与world重合，旧联调图保留(-8,-2)出生点。
        forest_scene=bool(os.environ.get('FOREST_TERRAIN_SCENE'))
        default_values=('1','0','0') if forest_scene else ('-7','-2','0')
        self.values = [tk.StringVar(value=value) for value in default_values]
        self.entries = []
        for row, (label, variable, unit) in enumerate(zip(('目标 X', '目标 Y', '最终朝向'), self.values, ('米', '米', '度'))):
            ttk.Label(box, text=label).grid(row=row, column=0, sticky='w', pady=6)
            entry = ttk.Entry(box, textvariable=variable, width=21)
            entry.grid(row=row, column=1, padx=12, sticky='ew')
            ttk.Label(box, text=unit).grid(row=row, column=2, sticky='w')
            self.entries.append(entry)
        box.columnconfigure(1, weight=1)
        self.preview_text = tk.StringVar(value='候选目标：等待输入')
        self.preview_label = ttk.Label(frame, textvariable=self.preview_text, wraplength=520)
        self.preview_label.pack(anchor='w', fill='x', pady=(8, 0))
        ttk.Label(frame, text='场景圆环 + 箭头：绿＝通过当前检查；红＝不允许停车；黄＝未知。\n仅预览，点击“前往目标”后才会运动。', wraplength=520).pack(anchor='w')
        # 连续输入只在下一次界面刷新时发布完整候选值；负号等中间值隐藏标记。
        self.preview_due = 0.
        for variable in self.values:
            variable.trace_add('write', self.preview_changed)
        row = ttk.Frame(frame)
        row.pack(fill='x', pady=10)
        ttk.Button(row, text='填入当前位置', command=self.fill_current).pack(side='left')
        ttk.Button(row, text='填入前方 1 米', command=self.fill_forward).pack(side='left', padx=10)
        actions = ttk.Frame(frame)
        actions.pack(fill='x', pady=4)
        self.go_button = ttk.Button(actions, text='前往目标 / 替换目标', style='Go.TButton', command=self.send)
        self.go_button.pack(side='left', fill='x', expand=True)
        self.stop_button = ttk.Button(actions, text='停止导航', style='Go.TButton', command=node.stop)
        self.stop_button.pack(side='left', padx=(12, 0))
        self.progress = tk.StringVar(value='导航状态：等待就绪')
        ttk.Label(frame, textvariable=self.progress).pack(anchor='w', pady=(14, 5))
        self.message = tk.StringVar(value=node.event)
        ttk.Label(frame, textvariable=self.message, wraplength=475).pack(anchor='w', fill='x')
        hint=('当前林地出生点约为 X=0、Y=0；默认目标 1、0 为前方约 1 米。' if forest_scene
              else '当前出生点约为 X=-8、Y=-2；默认目标 -7、-2 为前方约 1 米。')
        ttk.Label(frame, text=hint, wraplength=475, foreground='#666666').pack(anchor='w', pady=(14, 0))
        root.bind('<Return>', lambda event:self.send())
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.tick()

    def fill_current(self):
        if self.node.pose is not None:
            x, y, yaw = self.node.pose
            self.fill(x, y, math.degrees(yaw))

    def preview_changed(self, *_):
        self.preview_due = 0.

    def update_preview(self):
        now = time.monotonic()
        if now < self.preview_due:
            return
        self.preview_due = now + .2
        # 定期重新检查最新地图，目标不变但新观测发现障碍时也必须改变颜色。
        result = evaluate_preview(self.node.goal_validator, [v.get() for v in self.values])
        self.node.preview_publisher.publish(String(data=json.dumps(result, ensure_ascii=False, allow_nan=False)))
        color = {'allowed': '#137333', 'blocked': '#b3261e', 'unknown': '#946000', 'invalid': '#666666'}[result['status']]
        self.preview_label.configure(foreground=color)
        self.preview_text.set('候选目标：' + result['reason'])

    def fill_forward(self):
        if self.node.pose is not None:
            x, y, yaw = self.node.pose
            self.fill(x+math.cos(yaw), y+math.sin(yaw), math.degrees(yaw))

    def fill(self, x, y, yaw):
        for variable, value in zip(self.values, (x, y, yaw)):
            variable.set(f'{value:.3f}')

    def send(self):
        try:
            self.node.send_target(*(variable.get() for variable in self.values))
        except (ValueError, RuntimeError) as exc:
            self.node.event = str(exc) if isinstance(exc, RuntimeError) else '请输入有效数字，例如 X=-7、Y=-2、朝向=0。'

    def tick(self):
        if self.closed or not rclpy.ok():
            return
        self.node.poll()
        self.update_preview()
        ready = self.node.ready
        self.connection.set('已就绪，可以发送目标' if ready else '正在等待仿真、定位和导航服务就绪……')
        self.connection_label.configure(foreground='#137333' if ready else '#a45e00')
        self.go_button.configure(state='normal' if ready and self.node.pending_goal is None else 'disabled')
        if self.node.pose is not None:
            x, y, yaw = self.node.pose
            self.position.set(f'当前位置：X {x:.2f} 米    Y {y:.2f} 米    朝向 {math.degrees(yaw):.1f}°')
        # Action结果先于任务状态回调到达时，直接采用更具体的最终状态，
        # 避免界面短暂同时显示“正在前往”和“已到达”。
        record = self.node.records.current
        task_state = record['state'] if record else self.node.task.get('state')
        state = self.STATE_NAMES.get(task_state, '等待就绪')
        remaining = record.get('distance_remaining_m') if record else None
        suffix = f'    剩余路径约 {remaining:.2f} 米' if remaining is not None else ''
        task_id = record['task_id'][:8] if record else '无'
        # 新请求被拒绝时，之前的有效任务可能仍在执行，必须同时告知用户。
        active = self.node.records.snapshot()['active_tasks']
        other = [r for r in active if r['task_id'] != (record or {}).get('task_id')]
        previous = '；上一任务仍执行/取消中' if other else ''
        phase=PHASE_NAMES.get(self.node.mission.get('phase'),'') if self.node.mission_received and time.monotonic()-self.node.mission_received<1. else ''
        if self.node.mission.get('phase')=='arrived' and self.node.mission.get('parking_status')=='verified':phase='已到达并通过停稳检查'
        phase_text=f'；{phase}' if phase else ''
        self.progress.set(f'任务 {task_id}：{state}{suffix}{previous}{phase_text}')
        reason = (record.get('reason') if record else '') or self.node.task.get('reason', '')
        self.message.set(f'{self.node.event}\n{reason}' if reason else self.node.event)
        self.root.after(30, self.tick)

    def close(self):
        if not self.closed:
            self.closed = True
            self.node.closing = True
            self.node.stop()
            # 留给异步取消请求短暂处理时间；GUI 不等待整个物理制动过程。
            deadline = time.monotonic()+.4
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=.02)
            self.root.destroy()


def main():
    rclpy.init()
    node = CoordinateController()
    root = None
    panel = None
    try:
        root = tk.Tk()
        panel = ControllerPanel(root, node)
        root.mainloop()
    except KeyboardInterrupt:
        if panel is not None:
            panel.close()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
