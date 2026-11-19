"""FAR的估计输入：连续LIO坐标、真实测量地形；不订阅仿真真值。

FAR内部固定在lio_odom，地图平面修正由任务适配层转换，修正时清图结束旧任务。
"""
import copy
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from .lio_navigation import control_pose


class FarInputs(Node):
    def __init__(self):
        super().__init__('forest_far_inputs',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.pub=self.create_publisher(Odometry,'/far/odometry',10)
        self.sub=self.create_subscription(Odometry,'/lio/odometry',self.odom,10)

    def odom(self,msg):
        if msg.header.frame_id!='lio_odom' or msg.child_frame_id!='lio_imu':return
        p,q,yaw=control_pose([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        out=copy.deepcopy(msg);out.child_frame_id='far_control_root'
        out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,p)
        out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,q)
        self.pub.publish(out)


def main():
    rclpy.init();node=FarInputs()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
