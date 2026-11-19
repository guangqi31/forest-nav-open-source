"""坐标导航客户端；激光模式通过上层任务入口，其子路径与重规划仍交给Nav2。"""
import argparse
import json
import math
import os
import time
import rclpy
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from .navigation_client import NavigationClient
from .goal_validation import GoalMapValidator,GoalRejected
from .task_reporter import TaskReporter


class TaskClient(NavigationClient):
    def __init__(self):
        super().__init__()
        self.navigator = ActionClient(self, NavigateToPose, '/navigation/navigate_to_goal' if os.environ.get('FOREST_NAV_MAP_MODE')=='lidar' else '/navigate_to_pose')
        self.feedback = None
        self.records = TaskReporter(self, self.get_name())
        self.goal_validator = GoalMapValidator(self)

    def submit(self, goal):
        task_id = self.records.begin((float(goal[0]), float(goal[1]), float(goal[2]) if len(goal) == 3 else 0.))
        try:
            return self._submit(goal, task_id)
        except Exception as exc:
            if self.records.records[task_id]['state'] != 'rejected':
                self.records.update(task_id, state='rejected' if hasattr(exc, 'reason') else 'failed', reason=str(exc),reason_code=getattr(exc,'reason','submit_failed'))
            raise

    def _submit(self, goal, task_id):
        self.wait_ready()
        lifecycle = self.create_client(GetState, ('/controller_server/get_state' if os.environ.get('FOREST_NAV_PLANNER')=='far' else '/bt_navigator/get_state'))
        try:
            while rclpy.ok():
                if lifecycle.wait_for_service(timeout_sec=.5):
                    if self.future(lifecycle.call_async(GetState.Request())).current_state.id==3:break
        finally:self.destroy_client(lifecycle)
        while rclpy.ok() and not self.navigator.wait_for_server(timeout_sec=.5):pass
        # 与坐标面板共用实际代价话题的终点准入，拒绝后不发送Action。
        self.goal_validator.check(goal,allow_unobserved=os.environ.get('FOREST_NAV_MAP_MODE')=='lidar')
        request = NavigateToPose.Goal()
        request.pose = PoseStamped()
        request.pose.header.frame_id = 'map'
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x, request.pose.pose.position.y = map(float, goal[:2])
        yaw = float(goal[2]) if len(goal) == 3 else 0.
        request.pose.pose.orientation.z, request.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        handle = self.future(self.navigator.send_goal_async(request, feedback_callback=lambda msg: self.on_feedback(msg, task_id)))
        if not handle.accepted:
            self.records.update(task_id, state='rejected', reason='Nav2拒绝目标')
            raise RuntimeError('坐标导航目标被拒绝')
        handle._forest_task_id = task_id
        self.records.accepted(task_id, bytes(handle.goal_id.uuid).hex())
        handle._forest_result_future = handle.get_result_async()
        handle._forest_result_future.add_done_callback(lambda future: self.on_task_result(future, task_id))
        return handle

    def on_task_result(self, future, task_id):
        try:
            response = future.result()
            self.records.result(task_id, response.status,
                getattr(response.result, 'error_code', None), getattr(response.result, 'error_msg', ''))
        except Exception as exc:
            self.records.update(task_id, state='failed', reason=str(exc))

    def on_feedback(self, message, task_id=None):
        feedback = message.feedback
        self.feedback = dict(distance_remaining_m=feedback.distance_remaining,
            navigation_time_sec=feedback.navigation_time.sec+feedback.navigation_time.nanosec/1e9,
            number_of_recoveries=feedback.number_of_recoveries)

        if task_id is not None:
            self.records.update(task_id, state='navigating',
                distance_remaining_m=float(feedback.distance_remaining),
                pose=self.samples[-1][1:4] if self.samples else None)

    def wait_task(self, handle, timeout=0., cancel_after=None):
        result = handle._forest_result_future
        start = time.monotonic()
        cancel_sent = False
        try:
            while not result.done():
                rclpy.spin_once(self, timeout_sec=.02)
                elapsed = time.monotonic()-start
                if timeout>0 and elapsed > timeout:
                    raise TimeoutError('导航任务超时')
                if cancel_after is not None and elapsed >= cancel_after and not cancel_sent:
                    self.records.update(handle._forest_task_id, state='canceling', reason='用户请求取消')
                    self.future(handle.cancel_goal_async())
                    cancel_sent = True
            self.on_task_result(result, handle._forest_task_id)
            return dict(action_status=result.result().status, elapsed_sec=time.monotonic()-start,
                        cancel_requested=cancel_sent, feedback=self.feedback,
                        task=self.records.records.get(handle._forest_task_id))
        except TimeoutError:
            self.records.update(handle._forest_task_id,state='canceling',reason='用户指定的等待时限已到')
            self.future(handle.cancel_goal_async())
            raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal', type=float, nargs='+', required=True)
    parser.add_argument('--timeout', type=float, default=0., help='默认0：无限等待；正数仅用于显式限定本次客户端等待')
    parser.add_argument('--cancel-after', type=float)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout<0:parser.error('--timeout 必须非负且有限，0表示无限等待')
    if len(args.goal) not in (2, 3) or not all(math.isfinite(v) for v in args.goal):
        parser.error('--goal 需要有限的 x y [yaw]，单位为米、弧度')
    rclpy.init()
    node = TaskClient()
    try:
        handle = node.submit(args.goal)
        print('目标已接收，Nav2 开始规划、跟踪并周期性重规划', flush=True)
        report = node.wait_task(handle, args.timeout, args.cancel_after)
        node.spin_for(.5)
        report['goal'] = args.goal
        report['final_pose'] = node.samples[-1][1:4] if node.samples else None
        node.save('navigation', report)
        print(json.dumps(report, ensure_ascii=False), flush=True)
        expected = GoalStatus.STATUS_CANCELED if args.cancel_after is not None else GoalStatus.STATUS_SUCCEEDED
        if report['action_status'] != expected:
            raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
