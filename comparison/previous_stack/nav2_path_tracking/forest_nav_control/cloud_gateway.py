"""在线点云入口：Isaac真实相机射线命中或外部PointCloud2 → map坐标点云。

外部数据使用消息采集时刻查询TF，不使用“最新TF”替代历史变换。仿真UDP
数据已经位于world=map，来源为实际传感器命中，不读取完整场景几何。
"""
import os
import socket
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from .cloud_wire import CloudAssembler, PORT


class CloudGateway(Node):
    def __init__(self):
        super().__init__('forest_cloud_gateway',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.pub=self.create_publisher(PointCloud2,'/perception/terrain_points_map',qos_profile_sensor_data)
        self.source=os.environ.get('FOREST_NAV_CLOUD_SOURCE','sim_camera')
        self.last_stamp=0.;self.assembler=CloudAssembler()
        if self.source=='sim_camera':
            self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            self.sock.bind(('127.0.0.1',int(os.environ.get('FOREST_NAV_CLOUD_PORT',PORT))))
            self.sock.setblocking(False)
            # 不依赖仿真时钟推进来收包，避免启动时钟尚未到达的等待环。
            from rclpy.clock import Clock,ClockType
            self.timer=self.create_timer(.1,self.receive,clock=Clock(clock_type=ClockType.STEADY_TIME))
        elif self.source=='external':
            self.sub=self.create_subscription(PointCloud2,'/sensors/terrain_points',self.external,qos_profile_sensor_data)
        else: raise ValueError('点云来源应为sim_camera或external')

    def publish(self,points,stamp):
        if self.get_clock().now().nanoseconds/1e9 < self.last_stamp-.5:
            self.last_stamp=0.
        if stamp<=self.last_stamp:return
        self.last_stamp=stamp
        points=np.asarray(points,dtype=np.float32).reshape(-1,3)
        points=points[np.isfinite(points).all(axis=1)]
        header=Header(frame_id='map',stamp=Time(seconds=stamp).to_msg())
        self.pub.publish(point_cloud2.create_cloud_xyz32(header,points.tolist()))

    def receive(self):
        latest=None
        for _ in range(20):
            try: data,_=self.sock.recvfrom(65535)
            except BlockingIOError:break
            try:
                frame=self.assembler.feed(data)
                if frame is not None:latest=frame
            except ValueError:continue
        if latest:self.publish(latest[1],latest[0])

    def external(self,msg):
        try:
            stamp=Time.from_msg(msg.header.stamp).nanoseconds/1e9
            age=self.get_clock().now().nanoseconds/1e9-stamp
            if stamp<=0 or not math.isfinite(stamp):raise ValueError('点云时间戳无效')
            points=np.asarray(list(point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)),dtype=np.float64).reshape(-1,3)
            if msg.header.frame_id!='map':
                transform=self.tf.lookup_transform('map',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
                q=transform.rotation;x,y,z,w=q.x,q.y,q.z,q.w
                matrix=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
                points=points@matrix.T+np.array([transform.translation.x,transform.translation.y,transform.translation.z])
            self.publish(points,stamp)
        except Exception as exc:
            self.get_logger().warning('丢弃点云：'+str(exc),throttle_duration_sec=2.)

    def destroy_node(self):
        if hasattr(self,'sock'):self.sock.close()
        return super().destroy_node()


def main():
    rclpy.init();node=CloudGateway()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
