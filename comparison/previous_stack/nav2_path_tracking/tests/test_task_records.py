"""任务状态的顺序、取消竞态与ROS话题传输测试；不启动物理仿真。"""
import json
import time
import unittest
from unittest.mock import patch, PropertyMock
from rclpy.action import ActionServer
from nav2_msgs.action import NavigateToPose
from concurrent.futures import Future
from types import SimpleNamespace as NS
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from forest_nav_control.task_records import TaskRecords
from forest_nav_control.task_reporter import TaskReporter
from forest_nav_control.coordinate_controller import CoordinateController


def completed(value):
    future = Future()
    future.set_result(value)
    return future


class TaskRecordTests(unittest.TestCase):
    def setUp(self):
        self.records = TaskRecords('test')

    def test_arrival_is_not_parking_proof(self):
        task = self.records.begin((1, 2, .3))
        self.records.accepted(task, 'nav-id')
        self.records.result(task, 4)
        self.assertEqual(self.records.current['state'], 'succeeded')
        self.assertEqual(self.records.current['parking_status'], 'unverified')
        self.assertEqual(self.records.current['goal']['yaw_rad'], .3)

    def test_rejected_replacement_keeps_old_active(self):
        old = self.records.begin((1, 0, 0))
        self.records.accepted(old, 'old')
        new = self.records.begin((2, 0, 0))
        self.records.update(new, state='rejected', reason='地图禁行')
        self.assertEqual(self.records.snapshot()['active_tasks'][0]['task_id'], old)
        self.assertIsNone(self.records.records[old]['replaced_by'])

    def test_old_result_does_not_replace_current(self):
        old = self.records.begin((1, 0, 0))
        self.records.accepted(old, 'old')
        new = self.records.begin((2, 0, 0))
        self.records.accepted(new, 'new')
        self.records.result(old, 5)
        self.assertEqual(self.records.current['task_id'], new)
        self.assertEqual(self.records.records[old]['replaced_by'], new)
        self.assertEqual(self.records.records[old]['action_status'], 5)

    def test_delayed_feedback_cannot_undo_cancel_or_result(self):
        task = self.records.begin((1, 0, 0))
        self.records.update(task, state='canceling')
        self.records.update(task, state='navigating')
        self.assertEqual(self.records.current['state'], 'canceling')
        self.records.result(task, 5)
        self.records.update(task, state='navigating')
        self.assertEqual(self.records.current['state'], 'canceled')

    def test_delayed_acceptance_does_not_replace_new_task(self):
        old = self.records.begin((1, 0, 0))
        new = self.records.begin((2, 0, 0))
        self.records.accepted(new, 'new')
        self.records.accepted(old, 'old')
        self.assertIsNone(self.records.records[new]['replaced_by'])

    def test_error_details_are_preserved(self):
        task = self.records.begin((1, 0, 0))
        self.records.result(task, 6, 42, '路径不可达')
        self.assertEqual(self.records.current['error_code'], 42)
        self.assertEqual(self.records.current['reason'], '路径不可达')


class RosFeedbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = CoordinateController()

    def tearDown(self):
        self.node.destroy_node()

    def test_late_old_result_is_history_only(self):
        old = self.node.records.begin((1, 0, 0))
        new = self.node.records.begin((2, 0, 0))
        self.node.task_tokens = {1:old, 2:new}
        self.node.generation = 2
        self.node.event = '新目标执行中'
        self.node.on_result(completed(NS(status=5, result=NS())), 1)
        self.assertEqual(self.node.event, '新目标执行中')
        self.assertEqual(self.node.records.current_id, new)
        self.assertEqual(self.node.records.records[old]['state'], 'canceled')

    def test_late_goal_after_stop_is_canceled(self):
        task = self.node.records.begin((1, 0, 0))
        self.node.task_tokens = {1:task}
        self.node.generation = 2  # 用户已按停止，使旧提交token失效。
        cancellations = []
        result = Future()
        handle = NS(accepted=True, goal_id=NS(uuid=[1]*16),
            get_result_async=lambda: result, cancel_goal_async=lambda: cancellations.append(True))
        self.node.on_goal(completed(handle), 1)
        self.assertEqual(cancellations, [True])
        self.assertEqual(self.node.records.current['state'], 'canceling')
        result.set_result(NS(status=5, result=NS()))
        self.assertEqual(self.node.records.current['state'], 'canceled')

    def test_nav2_rejection_is_recorded(self):
        task = self.node.records.begin((1, 0, 0))
        self.node.task_tokens = {1:task}
        self.node.generation = 1
        self.node.on_goal(completed(NS(accepted=False)), 1)
        self.assertEqual(self.node.records.current['state'], 'rejected')

    def test_real_action_result_reaches_records(self):
        # 只替代Nav2服务端和启动就绪判断；真实使用ROS Action序列化、反馈与结果通道。
        # 该用例验证接口，不冒充真实机器人运动测试。
        server_node = Node('task_test_navigator')
        def execute(handle):
            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = .2
            handle.publish_feedback(feedback)
            handle.succeed()
            return NavigateToPose.Result()
        server = ActionServer(server_node, NavigateToPose, '/navigate_to_pose', execute)
        try:
            self.assertTrue(self.node.navigator.wait_for_server(timeout_sec=3))
            with patch.object(CoordinateController, 'ready', new_callable=PropertyMock, return_value=True):
                self.node.send_target(1, 0, 90)
            deadline = time.monotonic()+4
            while self.node.records.current['state'] not in ('succeeded', 'failed') and time.monotonic()<deadline:
                rclpy.spin_once(server_node, timeout_sec=.01)
                rclpy.spin_once(self.node, timeout_sec=.01)
            record = self.node.records.current
            self.assertEqual(record['state'], 'succeeded')
            self.assertEqual(record['action_status'], 4)
            self.assertEqual(record['parking_status'], 'unverified')
            self.assertAlmostEqual(record['goal']['yaw_rad'], 1.57079632679)
            self.assertTrue(record['goal_id'])
        finally:
            server.destroy()
            server_node.destroy_node()

    def test_task_records_ros_delivery(self):
        received = []
        observer = Node('task_record_observer')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = observer.create_subscription(String, '/navigation/task_records',
            lambda msg: received.append(json.loads(msg.data)), qos)
        task = self.node.records.begin((3, 4, .5))
        deadline = time.monotonic()+3
        try:
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(observer, timeout_sec=.05)
            self.assertTrue(received, '未收到真实ROS话题')
            self.assertEqual(received[-1]['current_task']['task_id'], task)
            self.assertEqual(received[-1]['current_task']['goal']['x'], 3)
        finally:
            observer.destroy_node()


if __name__ == '__main__':
    unittest.main()
