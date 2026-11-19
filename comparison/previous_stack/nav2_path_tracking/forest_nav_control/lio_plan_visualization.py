"""仅将新map路线投到Isaac世界用于画线；真值只出现在显示支路，不返回导航。

一次初始位姿配对确定world←lio_odom，后续用显式map←odom修正反变换路线。
该节点只输出/navigation/world_plan；规划器、控制器、地图均不订阅这个话题。
"""
import copy
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import String
from .sensor_contract import imu_pose_from_lidar,rotation
from .goal_preview import preview_file


def world_points(xy,correction,world_rotation,world_translation):
    x,y,yaw=correction;c,s=math.cos(yaw),math.sin(yaw)
    odom=(np.asarray(xy)-[x,y])@np.array([[c,-s],[s,c]])
    return np.c_[odom,np.zeros(len(odom))]@world_rotation.T+world_translation


class PlanVisualization(Node):
    def __init__(self):
        super().__init__('forest_lio_plan_visualization')
        self.truth={};self.estimated={};self.world_rotation=None;self.world_translation=None
        self.correction=[0.,0.,0.];self.path=None;self.generation=None
        self.preview=None;self.preview_received=None
        self.pub=self.create_publisher(Path,'/navigation/world_plan',1)
        self.subs=[self.create_subscription(Odometry,'/lidar/ground_truth_odometry',self.on_truth,20),
            self.create_subscription(Odometry,'/lio/odometry',self.on_estimate,20),
            self.create_subscription(String,'/lio/mapping_status',self.on_mapping,10),
            self.create_subscription(Path,'/plan',self.on_path,1),
            self.create_subscription(String,'/navigation/goal_preview',self.on_preview,1)]
        # 独立显示文件跨越 ROS/Isaac 的 Python 版本边界；不进入速度指令协议。
        self.preview_timer=self.create_timer(.2,self.draw_preview)

    def on_preview(self,msg):
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict):return
            if data.get('visible'):
                if data.get('status') not in ('allowed','blocked','unknown'):return
                if not all(math.isfinite(float(data[k])) for k in ('x','y','yaw')):return
            self.preview=data;self.preview_received=time.monotonic()
            self.draw_preview()
        except (ValueError,TypeError,KeyError):pass

    def draw_preview(self):
        """与路线使用同一 world←map 显示变换；导航只读取原 map 数据。"""
        now=time.monotonic()
        result=dict(visible=False,sent_monotonic=now)
        if (self.preview and self.preview_received is not None
                and now-self.preview_received<1. and self.world_rotation is not None
                and self.preview.get('visible')):
            result.update(self.preview)
            point=world_points([[result['x'],result['y']]],self.correction,
                               self.world_rotation,self.world_translation)[0]
            # 将 map 平面朝向也转换到 Isaac 世界，避免坐标修正后箭头旋转错误。
            angle=result['yaw']-self.correction[2]
            direction=self.world_rotation@np.array([math.cos(angle),math.sin(angle),0.])
            result.update(x=float(point[0]),y=float(point[1]),z=float(point[2]),
                          yaw=math.atan2(direction[1],direction[0]))
        path=preview_file()
        temporary=path.with_suffix('.tmp')
        temporary.write_text(json.dumps(result,ensure_ascii=False,allow_nan=False))
        temporary.replace(path)

    @staticmethod
    def record(msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        return msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,np.array([p.x,p.y,p.z,q.x,q.y,q.z,q.w])

    def on_truth(self,msg):
        stamp,pose=self.record(msg);self.truth[stamp]=imu_pose_from_lidar(pose);self.align()

    def on_estimate(self,msg):
        stamp,pose=self.record(msg);self.estimated[stamp]=pose;self.align()

    def align(self):
        if self.world_rotation is None:
            for stamp,pose in self.estimated.items():
                key=next((stamp+d for d in (0,-1,1,-2,2) if stamp+d in self.truth),None)
                if key is None:continue
                truth=self.truth[key];self.world_rotation=rotation(truth[3:])@rotation(pose[3:]).T
                self.world_translation=truth[:3]-self.world_rotation@pose[:3];self.draw();break
        for cache in (self.truth,self.estimated):
            while len(cache)>30:cache.pop(next(iter(cache)))

    def on_mapping(self,msg):
        data=json.loads(msg.data);generation=data.get('reset_generation')
        if self.generation is not None and self.generation!=generation:
            self.path=None;empty=Path();empty.header.frame_id='isaac_world';self.pub.publish(empty)
        self.generation=generation;self.correction=data.get('map_to_odom',[0.,0.,0.])

    def on_path(self,msg):
        if msg.header.frame_id=='map':self.path=msg;self.draw()

    def draw(self):
        if self.path is None or self.world_rotation is None:return
        result=copy.deepcopy(self.path);result.header.frame_id='isaac_world'
        if result.poses:
            points=world_points([[p.pose.position.x,p.pose.position.y] for p in result.poses],
                self.correction,self.world_rotation,self.world_translation)
            for pose,point in zip(result.poses,points):
                pose.header.frame_id='isaac_world';pose.pose.position.x=float(point[0]);pose.pose.position.y=float(point[1])
        self.pub.publish(result)


def main():
    rclpy.init();node=PlanVisualization()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
