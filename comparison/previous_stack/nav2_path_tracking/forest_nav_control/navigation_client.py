"""提交目标：等待真实反馈、调用全局规划、执行 FollowPath、监视并保存轨迹。"""
import argparse
import json
import math
import os
from pathlib import Path
import time
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath
from dwb_msgs.msg import LocalPlanEvaluation
from nav_msgs.msg import Odometry, Path as PathMessage
from std_msgs.msg import String
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import Buffer, TransformListener

ROOT=Path(__file__).resolve().parents[1]


class NavigationClient(Node):
    def __init__(self):
        super().__init__('forest_navigation_client',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.planner=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.follower=ActionClient(self,FollowPath,'/follow_path')
        self.odom,self.status=None,None
        self.status_received=None
        self.samples=[]
        self.raw_commands=[]
        self.adapted_commands=[]
        self.current_path=None
        self.last_evaluation=None
        # 详细候选轨迹较大，仅排查评分问题时启用，避免常规运行额外传输负担。
        self.evaluation_sub=None
        if os.environ.get("FOREST_NAV_DEBUG_DWB", "0")=="1":
            self.evaluation_sub=self.create_subscription(LocalPlanEvaluation,"/evaluation",self._evaluation,1)
        self.odom_sub=self.create_subscription(Odometry,'/odom',self._odom,20)
        self.status_sub=self.create_subscription(String,'/navigation/bridge_status',self._status,20)
        self.raw_sub=self.create_subscription(Twist,'/cmd_vel',lambda msg:self.raw_commands.append(
            (time.monotonic(),msg.linear.x,msg.linear.y,msg.angular.z)),20)
        self.adapted_sub=self.create_subscription(Twist,'/cmd_vel_adapted',lambda msg:self.adapted_commands.append(
            (time.monotonic(),msg.linear.x,msg.linear.y,msg.angular.z)),20)
        self.path_pub=self.create_publisher(PathMessage,'/navigation/path',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf=Buffer()
        self.listener=TransformListener(self.tf,self)

    def _evaluation(self,msg):
        self.last_evaluation=msg

    def _odom(self,msg):
        self.odom=msg
        pose=msg.pose.pose
        yaw=2*math.atan2(pose.orientation.z,pose.orientation.w)
        x,y=pose.position.x,pose.position.y
        if os.environ.get('FOREST_NAV_LOCALIZATION')=='lio':
            # /odom保持连续局部坐标；任务记录和目标使用map，显式修正后不能继续混用。
            try:
                transform=self.tf.lookup_transform('map','odom',rclpy.time.Time()).transform
                angle=2*math.atan2(transform.rotation.z,transform.rotation.w)
                c,s=math.cos(angle),math.sin(angle)
                x,y=c*x-s*y+transform.translation.x,s*x+c*y+transform.translation.y
                yaw=math.atan2(math.sin(yaw+angle),math.cos(yaw+angle))
            except Exception:return  # 等待有效变换，不把局部坐标冒充全局任务轨迹。
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        self.samples.append([stamp,x,y,yaw,
            msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.angular.z])

    def _status(self,msg):
        self.status=json.loads(msg.data)
        self.status_received=time.monotonic()

    def spin_for(self,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.02)

    def future(self,future,timeout=None):
        while rclpy.ok() and not future.done():rclpy.spin_once(self,timeout_sec=.05)
        return future.result()

    def wait_ready(self,timeout=None):
        while rclpy.ok():
            rclpy.spin_once(self,timeout_sec=.1)
            if self.odom is not None and self.tf.can_transform('map','base_link',rclpy.time.Time()):break
        for name in (('controller_server',) if os.environ.get('FOREST_NAV_PLANNER')=='far' else ('planner_server','controller_server')):
            client=self.create_client(GetState,f'/{name}/get_state')
            try:
                while rclpy.ok():
                    if client.wait_for_service(timeout_sec=.5) and self.future(client.call_async(GetState.Request())).current_state.id==3:break
            finally:self.destroy_client(client)
        for action in ([self.follower] if os.environ.get('FOREST_NAV_PLANNER')=='far' else [self.planner,self.follower]):
            while rclpy.ok() and not action.wait_for_server(timeout_sec=.5):pass

    def plan(self,goal):
        self.wait_ready()
        request=ComputePathToPose.Goal()
        request.use_start=False  # 使用实时定位，绝不复用第一阶段固定起点。
        request.planner_id='GridBased'
        request.goal=PoseStamped()
        request.goal.header.frame_id='map'
        request.goal.header.stamp=self.get_clock().now().to_msg()
        request.goal.pose.position.x,request.goal.pose.position.y=map(float,goal[:2])
        yaw=goal[2] if len(goal)>2 else 0.
        request.goal.pose.orientation.z,request.goal.pose.orientation.w=math.sin(yaw/2),math.cos(yaw/2)
        handle=self.future(self.planner.send_goal_async(request))
        if not handle.accepted:
            raise RuntimeError('全局规划请求被拒绝')
        result=self.future(handle.get_result_async())
        if result.status!=GoalStatus.STATUS_SUCCEEDED or len(result.result.path.poses)<2:
            raise RuntimeError(f'无法生成到目标的全局路径，状态 {result.status}')
        self.current_path=result.result.path
        self.path_pub.publish(self.current_path)
        return self.current_path

    def follow(self,path,timeout=0,cancel_after=None):
        request=FollowPath.Goal()
        request.path=path
        request.controller_id='FollowPath'
        request.goal_checker_id='goal_checker'
        handle=self.future(self.follower.send_goal_async(request))
        if not handle.accepted:
            raise RuntimeError('FollowPath 请求被拒绝')
        result_future=handle.get_result_async()
        started=time.monotonic()
        cancel_sent=False
        try:
            while not result_future.done():
                rclpy.spin_once(self,timeout_sec=.02)
                elapsed=time.monotonic()-started
                if timeout>0 and elapsed>timeout:
                    raise TimeoutError('路径执行超过等待时间')
                if cancel_after is not None and elapsed>=cancel_after and not cancel_sent:
                    response=self.future(handle.cancel_goal_async(),timeout=3)
                    if not response.goals_canceling:
                        raise RuntimeError('取消请求未被接受')
                    cancel_sent=True
            result=result_future.result()
            return {'action_status':result.status,'elapsed_sec':time.monotonic()-started,
                    'cancel_requested':cancel_sent}
        except (KeyboardInterrupt,TimeoutError):
            if not result_future.done():handle.cancel_goal_async()
            raise

    def save(self,name,report):
        result_dir=ROOT/'results'
        result_dir.mkdir(exist_ok=True)
        np.savetxt(result_dir/f'{name}_trajectory.csv',np.asarray(self.samples).reshape(-1,7),delimiter=',',
                   header='sim_time,x_m,y_m,yaw_rad,vx_mps,vy_mps,wz_radps',comments='')
        if self.last_evaluation is not None:
            from rosidl_runtime_py.convert import message_to_ordereddict
            (result_dir/f'{name}_evaluation.json').write_text(json.dumps(message_to_ordereddict(self.last_evaluation),indent=2))
        if self.current_path is not None:
            from rosidl_runtime_py.convert import message_to_ordereddict
            (result_dir/f'{name}_path.json').write_text(json.dumps(message_to_ordereddict(self.current_path),indent=2))
        (result_dir/f'{name}_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal',type=float,nargs='+',default=[-6.5,-2.,0.])
    parser.add_argument('--timeout',type=float,default=0)
    parser.add_argument('--cancel-after',type=float)
    args=parser.parse_args()
    if len(args.goal) not in (2,3) or not all(math.isfinite(v) for v in args.goal):
        parser.error('--goal 必须是有限的 x y 或 x y yaw')
    rclpy.init()
    node=NavigationClient()
    try:
        path=node.plan(args.goal)
        print(f'实时起点规划完成：{len(path.poses)} 个路径点；开始跟踪',flush=True)
        report=node.follow(path,args.timeout,args.cancel_after)
        node.spin_for(1.)
        report['final_pose']=node.samples[-1][1:4]
        node.save('navigation',report)
        expected=GoalStatus.STATUS_CANCELED if args.cancel_after is not None else GoalStatus.STATUS_SUCCEEDED
        if report['action_status']!=expected:
            raise RuntimeError(f'路径跟踪未达到预期状态：{report}')
        print(json.dumps(report,ensure_ascii=False),flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':
    main()
