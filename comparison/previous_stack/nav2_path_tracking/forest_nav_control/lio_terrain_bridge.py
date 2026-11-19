"""为独立CMU实例提供估计激光位姿，并明确其硬编码map输出实际属于lio_odom。

全程只订阅LIO估计，不使用真值位姿/真值点云。CMU分析在连续里程计系内进行，
因此地图刚体坐标修正不要求清空CMU的局部缓存。
"""
import copy
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
from .sensor_contract import extrinsics,rotation,quaternion_multiply


class LioTerrainBridge(Node):
    def __init__(self):
        super().__init__('lio_terrain_bridge')
        self.odom_pub=self.create_publisher(Odometry,'/lio/lidar_odometry',20)
        self.terrain_pub=self.create_publisher(PointCloud2,'/lio/terrain_map',5)
        self.odom_sub=self.create_subscription(Odometry,'/lio/odometry',self.on_odom,20)
        self.terrain_sub=self.create_subscription(PointCloud2,'/lio/terrain_raw',self.on_terrain,5)
        self.translation,self.quaternion=extrinsics()

    def on_odom(self,msg):
        if msg.header.frame_id!='lio_odom' or msg.child_frame_id!='lio_imu':return
        out=copy.deepcopy(msg);p=msg.pose.pose.position;q=msg.pose.pose.orientation
        quat=np.array([q.x,q.y,q.z,q.w])
        position=np.array([p.x,p.y,p.z])+rotation(quat)@self.translation
        quat=quaternion_multiply(quat,self.quaternion)
        out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,position)
        out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,quat)
        out.child_frame_id='lio_lidar'
        # FAST-LIO不提供本阶段可用的激光中心速度；CMU只消费pose，避免误传IMU中心twist。
        from geometry_msgs.msg import TwistWithCovariance
        out.twist=TwistWithCovariance()
        self.odom_pub.publish(out)

    def on_terrain(self,msg):
        if msg.header.frame_id!='map':raise ValueError('独立CMU输出与预期源码契约不符')
        msg.header.frame_id='lio_odom';self.terrain_pub.publish(msg)


def main():
    rclpy.init();node=LioTerrainBridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
