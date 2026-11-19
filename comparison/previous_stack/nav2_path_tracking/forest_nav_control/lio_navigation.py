"""LIO导航边界：接管map→odom和代价图，robot_localization输出odom/base_link。

不订阅真值位姿/速度/点云。lio_odom连续，map→odom只随显式地图修正变化；
base_link是控制根torso_link的水平投影，传感器完整姿态由base_link→imu_link保留。
"""
import os
import copy
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry,OccupancyGrid
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String,Bool
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from .sensor_contract import imu_settings,rotation,quaternion_multiply
from .map_snapshot import fingerprint


def control_pose(pose):
    """由安装外参去掉IMU偏置，返回连续坐标内控制根的完整位姿和水平航向。"""
    settings=imu_settings();inverse=np.asarray(settings['rotation_xyzw'])*[-1.,-1.,-1.,1.]
    q=quaternion_multiply(pose[3:],inverse)
    p=np.asarray(pose[:3])-rotation(q)@np.asarray(settings['offset_torso_m'])
    r=rotation(q);yaw=math.atan2(r[1,0],r[0,0])
    return p,q,yaw


class LioNavigation(Node):
    def __init__(self):
        super().__init__('forest_lio_navigation',parameter_overrides=[Parameter('use_sim_time',value=True)])
        # 当前策略以URDF根刚体作为速度控制基准；换模型必须显式重新适配，不能猜测为pelvis。
        urdf=Path(__file__).resolve().parents[2]/'InstinctLab/source/instinctlab/instinctlab/assets/resources/unitree_g1/urdf/g1_29dof_torsobase_popsicle.urdf'
        robot=ET.parse(urdf).getroot();children={j.find('child').get('link') for j in robot.findall('joint')}
        roots={link.get('name') for link in robot.findall('link')}-children
        if roots!={'torso_link'}:raise ValueError('控制根与IMU安装契约变化，请重新标定控制外参')
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        alias=TransformStamped(child_frame_id='lio_map');alias.header.frame_id='map';alias.transform.rotation.w=1.
        self.static.sendTransform(alias)
        # /odom与odom→base_link的唯一发布者为现成robot_localization。
        self.odom_pub=self.create_publisher(Odometry,'/lio/control_odometry',20)
        self.filtered=None;self.filtered_received=None
        self.localization_pub=self.create_publisher(String,'/navigation/localization_status',10)
        self.mapping_pub=self.create_publisher(String,'/navigation/mapping_status',10)
        self.stop=self.create_publisher(Bool,'/navigation/stop_requested',10)
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pubs=[self.create_publisher(OccupancyGrid,t,qos) for t in ('/traversability_map','/traversability_map_local')]
        self.maps=[{},{}];self.mapping={};self.mapping_received=None;self.source=None
        self.last_pose=None;self.last_stamp=None;self.last_odom_wall=None;self.velocity=np.zeros(3)
        self.latest=None;self.map_pose=None;self.generation=None;self.last_map_key=None
        self.native_active=False;self.error='等待LIO与一致地图快照'
        self.subs=[self.create_subscription(Odometry,'/lio/odometry',self.on_odom,20),
            self.create_subscription(Odometry,'/odom',self.on_filtered,20),
            self.create_subscription(String,'/lio/mapping_status',self.on_mapping,20),
            self.create_subscription(OccupancyGrid,'/lio/traversability_map',lambda m:self.on_map(m,0),qos),
            self.create_subscription(OccupancyGrid,'/lio/traversability_map_local',lambda m:self.on_map(m,1),qos),
            self.create_subscription(GoalStatusArray,('/follow_path/_action/status' if os.environ.get('FOREST_NAV_PLANNER')=='far' else '/navigate_to_pose/_action/status'),self.on_action,10)]
        self.timer=self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_filtered(self,msg):
        if msg.header.frame_id=='odom' and msg.child_frame_id=='base_link':
            self.filtered=msg;self.filtered_received=time.monotonic()

    def on_action(self,msg):
        if msg.status_list:
            latest=max(msg.status_list,key=lambda s:(s.goal_info.stamp.sec,s.goal_info.stamp.nanosec))
            self.native_active=latest.status in (1,2,3)

    def on_map(self,msg,index):
        if msg.header.frame_id!='lio_map' or len(msg.data)!=msg.info.width*msg.info.height:return
        key=fingerprint(msg);cache=self.maps[index];cache[key]=msg
        while len(cache)>3:cache.pop(next(iter(cache)))

    def on_mapping(self,msg):
        data=json.loads(msg.data);source=data.get('source')
        if source:
            source=tuple(source)
            if self.source is not None and source!=self.source:self.last_stamp=None
            self.source=source
        generation=data.get('reset_generation',0)
        self.generation=generation;self.mapping=data;self.mapping_received=time.monotonic()

    def on_odom(self,msg):
        if msg.header.frame_id!='lio_odom' or msg.child_frame_id!='lio_imu':return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        imu=np.array([p.x,p.y,p.z,q.x,q.y,q.z,q.w]);position,quat,yaw=control_pose(imu)
        current=np.array([position[0],position[1],yaw])
        if self.last_stamp is not None:
            dt=stamp-self.last_stamp
            delta=current-self.last_pose;delta[2]=math.atan2(math.sin(delta[2]),math.cos(delta[2]))
            # 速度来自连续LIO位姿差分，不使用指令或仿真真值。滤掉5Hz位姿轻微抖动。
            alpha=dt/(.15+dt);self.velocity+=(delta/dt-self.velocity)*alpha
        self.last_pose=current;self.last_stamp=stamp;self.last_odom_wall=time.monotonic();self.latest=(msg,imu,position,quat,yaw)
        odom=copy.deepcopy(msg);odom.header.frame_id='odom';odom.child_frame_id='base_link'
        odom.pose.pose.position.x=float(position[0]);odom.pose.pose.position.y=float(position[1]);odom.pose.pose.position.z=0.
        odom.pose.pose.orientation.x=0.;odom.pose.pose.orientation.y=0.
        odom.pose.pose.orientation.z=math.sin(yaw/2);odom.pose.pose.orientation.w=math.cos(yaw/2)
        c,s=math.cos(yaw),math.sin(yaw)
        odom.twist.twist.linear.x=float(c*self.velocity[0]+s*self.velocity[1])
        odom.twist.twist.linear.y=float(-s*self.velocity[0]+c*self.velocity[1])
        odom.twist.twist.angular.z=float(self.velocity[2]);self.odom_pub.publish(odom)
        # 传感器完整姿态仍可沿map→odom→base_link→imu_link→lidar_link查询。
        planar_q=np.array([0.,0.,math.sin(yaw/2),math.cos(yaw/2)])
        inverse=planar_q*[-1.,-1.,-1.,1.]
        rel_p=rotation(inverse)@(imu[:3]-np.array([position[0],position[1],0.]))
        rel_q=quaternion_multiply(inverse,imu[3:])
        sensor=TransformStamped(child_frame_id='imu_link');sensor.header.stamp=odom.header.stamp;sensor.header.frame_id='base_link'
        sensor.transform.translation.x,sensor.transform.translation.y,sensor.transform.translation.z=map(float,rel_p)
        sensor.transform.rotation.x,sensor.transform.rotation.y,sensor.transform.rotation.z,sensor.transform.rotation.w=map(float,rel_q)
        self.tf.sendTransform(sensor)

    def tick(self):
        wall=time.monotonic();now=self.get_clock().now().nanoseconds*1e-9
        state=self.mapping;digests=state.get('map_fingerprints',[])
        coherent=len(digests)==2 and all(d in cache for d,cache in zip(digests,self.maps))
        if coherent:
            key=(state.get('revision'),self.generation,*digests)
            if key!=self.last_map_key:
                for d,cache,pub in zip(digests,self.maps,self.map_pubs):
                    msg=copy.deepcopy(cache[d]);msg.header.frame_id='map';pub.publish(msg)
                self.last_map_key=key
        # 新鲜度只用于状态报告，数据发布、TF和任务执行不由此分支否决。
        lio_age=None if self.last_odom_wall is None else wall-self.last_odom_wall
        filtered_age=None if self.filtered_received is None else wall-self.filtered_received
        measurement_age=None if self.last_stamp is None else now-self.last_stamp
        lio_fresh=bool(lio_age is not None and 0.<=lio_age<1.
            and measurement_age is not None and -.1<=measurement_age<1.)
        filtered_fresh=bool(filtered_age is not None and 0.<=filtered_age<1.)
        ready=bool(coherent and lio_fresh and filtered_fresh)
        reason=('' if ready else 'waiting_coherent_map' if not coherent else
            'waiting_lio' if self.latest is None else 'lio_stale' if not lio_fresh else
            'filtered_odometry_stale')
        transform=state.get('map_to_odom',[0.,0.,0.]);x,y,angle=transform;c,s=math.cos(angle),math.sin(angle)
        if self.latest is not None:
            msg,imu,position,quat,yaw=self.latest
            # map→odom在两次显式修正之间是常量，提供0.05秒有效期，留在原监视器的未来容差内。
            # 这不延长LIO测量有效期：ready仍检查上面的原始last_stamp。
            tf=TransformStamped(child_frame_id='odom');tf.header.frame_id='map'
            from rclpy.duration import Duration
            tf.header.stamp=(self.get_clock().now()+Duration(seconds=.05)).to_msg()
            tf.transform.translation.x=float(x);tf.transform.translation.y=float(y)
            tf.transform.rotation.z=math.sin(angle/2);tf.transform.rotation.w=math.cos(angle/2);self.tf.sendTransform(tf)
            if self.filtered is not None:
                pose=self.filtered.pose.pose;position=pose.position;q=pose.orientation
                yaw=2*math.atan2(q.z,q.w)
                self.map_pose=[c*position.x-s*position.y+x,s*position.x+c*position.y+y,
                    math.atan2(math.sin(yaw+angle),math.cos(yaw+angle))]
        normalized=dict(state,source='cmu_lio',localization_source='fast_lio',ready=ready,
            session=None if not self.source else self.source[0],clock_epoch=None if not self.source else self.source[1],
            frame_id='map',reason=reason,
            lio_age_sec=lio_age,measurement_age_sec=measurement_age,
            filtered_odometry_age_sec=filtered_age,localization_fresh=lio_fresh and filtered_fresh,
            map_snapshot_coherent=coherent)
        self.mapping_pub.publish(String(data=json.dumps(normalized,ensure_ascii=False)))
        self.localization_pub.publish(String(data=json.dumps(dict(ready=ready,source='fast_lio',frame_id='map',
            pose=self.map_pose,stamp=self.last_stamp,session=None if not self.source else self.source[0],
            reset_generation=self.generation,reason=normalized['reason'],
            lio_age_sec=lio_age,measurement_age_sec=measurement_age,
            filtered_odometry_age_sec=filtered_age),ensure_ascii=False)))


def main():
    rclpy.init();node=LioNavigation()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
