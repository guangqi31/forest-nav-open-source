"""数值目标 → 标准NavigateToPose客户端；不做地图判断、任务重试或速度控制。"""
import argparse
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose

def main():
    p=argparse.ArgumentParser();p.add_argument('--goal',nargs='+',type=float,required=True);p.add_argument('--timeout',type=float,default=0.)
    args=p.parse_args()
    if len(args.goal) not in (2,3):p.error('--goal X Y [朝向角度]')
    rclpy.init();node=Node('navigation_goal_client');client=ActionClient(node,NavigateToPose,'/navigate_to_pose')
    try:
        while not client.wait_for_server(timeout_sec=1.):print('等待Nav2目标接口',flush=True)
        goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=node.get_clock().now().to_msg()
        goal.pose.pose.position.x,goal.pose.pose.position.y=args.goal[:2];yaw=math.radians(args.goal[2] if len(args.goal)>2 else 0.)
        goal.pose.pose.orientation.z=math.sin(yaw/2);goal.pose.pose.orientation.w=math.cos(yaw/2)
        future=client.send_goal_async(goal);rclpy.spin_until_future_complete(node,future);handle=future.result()
        if not handle.accepted:raise RuntimeError('Nav2拒绝目标')
        print('Nav2已接收目标',flush=True);result=handle.get_result_async();rclpy.spin_until_future_complete(node,result)
        print('Nav2结果状态:',result.result().status,flush=True)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
