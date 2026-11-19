"""FAST-LIO的传感器格式边界和隔离输出；不读取任何真值数据。

瞬时点云的time=0必须原样表达为timestamp=0，不能按方位角虚构点内时间。
上游输出的位姿是IMU位姿；只改坐标系名称，不添加真值平移或姿态修正。
"""
import json
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from .sensor_contract import extrinsics


def convert_cloud(msg):
    """只改字段布局。时间单位为秒；输出28字节点，与PCL内存对齐无关。"""
    points = point_cloud2.read_points(msg, field_names=('x','y','z','intensity','ring','time'))
    if not len(points) or not np.isfinite(points['time']).all() or np.any(points['time'] != 0):
        raise ValueError('当前适配器只接受非空的瞬时扫描(time全部为0)')
    dtype=np.dtype({'names':['x','y','z','intensity','ring','timestamp'],
        'formats':['<f4','<f4','<f4','<f4','<u2','<f8'],
        'offsets':[0,4,8,12,16,20],'itemsize':28})
    packed=np.zeros(points.size,dtype=dtype)
    for name in ('x','y','z','intensity','ring'):packed[name]=points[name].reshape(-1)
    fields=[PointField(name=n,offset=o,datatype=d,count=1) for n,o,d in
        [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('ring',16,4),('timestamp',20,8)]]
    return PointCloud2(header=msg.header,height=1,width=packed.size,fields=fields,
        is_bigendian=False,point_step=28,row_step=28*packed.size,data=packed.tobytes(),is_dense=msg.is_dense)


class LioAdapter(Node):
    def __init__(self):
        super().__init__('forest_lio_adapter')
        self.input_count=0;self.odom_count=0;self.last_odom_wall=None;self.source=None
        self.source_ready=False;self.fault='';self.last_status_wall=None
        self.points=self.create_publisher(PointCloud2,'/lio/input/points',20)
        self.odom=self.create_publisher(Odometry,'/lio/odometry',20)
        self.path=self.create_publisher(Path,'/lio/path',5)
        self.cloud=self.create_publisher(PointCloud2,'/lio/cloud_registered',5)
        self.map=self.create_publisher(PointCloud2,'/lio/map',2)
        self.status=self.create_publisher(String,'/lio/status',10)
        self.tf=TransformBroadcaster(self)
        self.static_tf=StaticTransformBroadcaster(self)
        t,q=extrinsics()
        mount=TransformStamped(child_frame_id='lio_lidar')
        mount.header.frame_id='lio_imu'
        mount.transform.translation.x,mount.transform.translation.y,mount.transform.translation.z=map(float,t)
        mount.transform.rotation.x,mount.transform.rotation.y,mount.transform.rotation.z,mount.transform.rotation.w=map(float,q)
        self.static_tf.sendTransform(mount)
        self.subs=[self.create_subscription(PointCloud2,'/lidar/points_synced',self.on_points,20),
            self.create_subscription(String,'/sensors/sync_status',self.on_source,10),
            self.create_subscription(Odometry,'/lio/raw/odometry',self.on_odom,20),
            self.create_subscription(Path,'/lio/raw/path',self.on_path,5),
            self.create_subscription(PointCloud2,'/lio/raw/cloud',lambda m:self.on_cloud(m,self.cloud),5),
            self.create_subscription(PointCloud2,'/lio/raw/map',lambda m:self.on_cloud(m,self.map),2)]
        self.timer=self.create_timer(.2,self.publish_status)

    def on_source(self,msg):
        data=json.loads(msg.data)
        source=(data.get('session'),data.get('clock_epoch'))
        # 桥先于仿真启动时会发送session=null的等待状态；它不是一个真实采集会话。
        if not source[0]:
            self.source_ready=False;self.last_status_wall=time.monotonic();return
        # 估计器不能跨仿真重启/时钟回跳拼接地图；必须由启动器整组重启。
        self.source=source;self.source_ready=bool(data.get('ready'))
        self.last_status_wall=time.monotonic()

    def on_points(self,msg):
        try:self.points.publish(convert_cloud(msg));self.input_count+=1;self.fault=''
        except ValueError as exc:self.fault=str(exc)

    def on_odom(self,msg):
        if not np.isfinite([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]).all():
            self.get_logger().warning('跳过非有限LIO位姿');return
        msg.header.frame_id='lio_odom';msg.child_frame_id='lio_imu'
        self.odom.publish(msg);self.odom_count+=1;self.last_odom_wall=time.monotonic()
        # 仅新增lio_odom→lio_imu分支，不接到map/odom/base_link，避免多发布者抢权。
        transform=TransformStamped(header=msg.header,child_frame_id='lio_imu')
        transform.transform.translation.x=msg.pose.pose.position.x
        transform.transform.translation.y=msg.pose.pose.position.y
        transform.transform.translation.z=msg.pose.pose.position.z
        transform.transform.rotation=msg.pose.pose.orientation
        self.tf.sendTransform(transform)

    def on_path(self,msg):
        msg.header.frame_id='lio_odom'
        for pose in msg.poses:pose.header.frame_id='lio_odom'
        self.path.publish(msg)

    def on_cloud(self,msg,publisher):
        msg.header.frame_id='lio_odom';publisher.publish(msg)

    def publish_status(self):
        now=time.monotonic()
        # 仅修正状态报告：收到过数据不等于当前仍有更新。
        # 沿用导航状态链的一秒新鲜度口径，不拦截点云/里程计、不发送停止或退出。
        age=None if self.last_odom_wall is None else now-self.last_odom_wall
        source_age=None if self.last_status_wall is None else now-self.last_status_wall
        fresh=age is not None and 0.<=age<1.
        source_fresh=source_age is not None and 0.<=source_age<1.
        ready=bool(self.source and self.source_ready and source_fresh and fresh and not self.fault)
        reason=('' if ready else self.fault or ('waiting_odometry' if age is None else
            'odometry_stale' if not fresh else 'sensor_source_not_ready'))
        self.status.publish(String(data=json.dumps(dict(version=1,ready=ready,fault=self.fault,reason=reason,
            odometry_age_sec=age,odometry_fresh=fresh,sensor_status_age_sec=source_age,
            input_clouds=self.input_count,odometry_count=self.odom_count,source=self.source,
            navigation_localization='lio' if os.environ.get('FOREST_NAV_LOCALIZATION')=='lio' else 'unchanged_truth',scan_model='instantaneous'))))


def main():
    rclpy.init();node=LioAdapter()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
