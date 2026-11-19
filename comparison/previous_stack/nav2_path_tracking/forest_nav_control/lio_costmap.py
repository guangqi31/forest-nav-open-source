"""旁路LIO地形代价地图：自动扩展、原始观测保留、平面坐标修正后事务重建。

公开话题独立于当前Nav2，绝不把真值点云补入地图，也不把未知填成自由。
坐标修正只提供消费接口；FAST-LIO本身没有在这里增加回环检测或漂移修正算法。
"""
import json
import math
import os
import signal
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from .terrain_rebuild import TerrainEvidence
from .map_snapshot import fingerprint


class LioCostmap(Node):
    def __init__(self):
        super().__init__('lio_costmap',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.classified=os.environ.get('FOREST_NAV_PLANNER','far')=='far'
        self.classified_clouds={}
        self.evidence=TerrainEvidence(classified=self.classified);self.source=None;self.lio_ready=False;self.lio_received=None
        self.last_clock=None;self.last_error='';self.last_correction=None;self.updates=0;self.rejected=0
        self.minimum_stamp=0.;self.fingerprints=[]
        self.last_save=0.;self.last_expire=0.;self.storage_error=''
        self.folder=Path(os.environ.get('FOREST_LIO_OUTPUT',str(Path(__file__).resolve().parents[2]/'forest_lidar_perception/results/lio_live')))/'costmaps'
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.global_pub=self.create_publisher(OccupancyGrid,'/lio/traversability_map',qos)
        self.local_pub=self.create_publisher(OccupancyGrid,'/lio/traversability_map_local',qos)
        self.status=self.create_publisher(String,'/lio/mapping_status',10)
        self.tf=TransformBroadcaster(self)
        self.subs=[
            self.create_subscription(String,'/lio/status',self.on_lio,10),
            self.create_subscription(String,'/lio/mapping/correction',self.on_correction,10)]
        if self.classified:
            self.subs.extend([
                self.create_subscription(PointCloud2,'/FAR_free_debug',lambda m:self.on_classified(m,0),5),
                self.create_subscription(PointCloud2,'/FAR_obs_debug',lambda m:self.on_classified(m,100),5)])
        else:
            self.subs.append(self.create_subscription(PointCloud2,'/lio/terrain_map',self.on_terrain,2))
        self.save=self.create_service(Trigger,'/lio/mapping/save',self.on_save)
        if os.environ.get('FOREST_NAV_LOCALIZATION')=='lio':
            self.nav_save=self.create_service(Trigger,'/mapping/save',self.on_save)
            self.nav_reset=self.create_service(Trigger,'/mapping/reset',self.on_reset)
        self.timer=self.create_timer(.2,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_maps()

    def reset_observations(self):
        generation=self.evidence.generation+1
        self.evidence=TerrainEvidence(classified=self.classified);self.evidence.generation=generation
        self.classified_clouds.clear()
        self.minimum_stamp=0.;self.updates=0
        self.publish_maps()

    def on_lio(self,msg):
        data=json.loads(msg.data);source=data.get('source')
        if not source or not source[0]:return
        source=tuple(source)
        if self.source is not None and source!=self.source:self.reset_observations()
        self.source=source;self.lio_ready=bool(data.get('ready'));self.lio_received=time.monotonic()

    def on_classified(self,msg,cost):
        """只栅格化 FAR 已判定的自由/障碍类别，不读取高度作二次判断。"""
        if msg.header.frame_id!='lio_odom':return
        self.classified_clouds[cost]=msg
        if len(self.classified_clouds)!=2:return
        clouds=self.classified_clouds;self.classified_clouds={}
        points=[]
        for value,cloud in clouds.items():
            xyz=point_cloud2.read_points_numpy(cloud,field_names=('x','y','z'),skip_nans=True)
            points.append(np.column_stack((xyz,np.full(len(xyz),value))))
        stamp=max(c.header.stamp.sec+c.header.stamp.nanosec/1e9 for c in clouds.values())
        self.observe_points(np.concatenate(points),stamp)

    def observe_points(self,points,stamp):
        if self.source is None or stamp<self.minimum_stamp:return
        if self.evidence.observe(points,stamp):
            self.updates+=1;self.last_error='';self.publish_maps()

    def on_terrain(self,msg):
        try:
            if self.source is None:return
            if msg.header.frame_id!='lio_odom':raise ValueError('地形观测必须位于连续lio_odom坐标')
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if stamp<self.minimum_stamp:return
            now=self.get_clock().now().nanoseconds*1e-9
            points=point_cloud2.read_points_numpy(msg,field_names=('x','y','z','intensity'),skip_nans=True)
            if self.evidence.observe(points,stamp):
                self.updates+=1;self.last_error='';self.publish_maps()
        except (ValueError,TypeError,AssertionError) as exc:
            self.rejected+=1;self.last_error=str(exc)

    def on_correction(self,msg):
        """带会话和单调版本的绝对修正；错误请求不改变地图、TF和版本。"""
        data={}
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict):raise ValueError('修正请求必须是JSON对象')
            if data.get('schema_version')!=1:raise ValueError('不支持的修正协议版本')
            if self.source is None or (data.get('session'),data.get('clock_epoch'))!=self.source:
                raise ValueError('修正会话/时钟代号不匹配')
            if data.get('frame_id')!='lio_map' or data.get('child_frame_id')!='lio_odom':
                raise ValueError('修正坐标必须是lio_map→lio_odom')
            if 'base_revision' in data and data['base_revision']!=self.evidence.revision:
                raise ValueError('坐标版本已变化，请读取最新状态后重试')
            if not self.evidence.groups:raise ValueError('还没有地形观测，不能提交修正')
            self.evidence.correct(data.get('map_to_odom'),data.get('revision'),self.get_clock().now().nanoseconds*1e-9)
            self.last_correction=dict(accepted=True,revision=self.evidence.revision,request_id=data.get('request_id'))
            self.publish_maps();self.publish_transform()
            self.save_maps()
        except (ValueError,TypeError,KeyError) as exc:
            self.last_correction=dict(accepted=False,requested_revision=data.get('revision') if isinstance(data,dict) else None,
                request_id=data.get('request_id') if isinstance(data,dict) else None,reason=str(exc))
        except OSError as exc:self.storage_error=str(exc)

    def publish_maps(self):
        fingerprints=[]
        for grid,publisher in [(self.evidence.global_grid,self.global_pub),(self.evidence.local_grid,self.local_pub)]:
            msg=OccupancyGrid();msg.header.frame_id='lio_map';msg.header.stamp=self.get_clock().now().to_msg()
            msg.info.resolution=grid.resolution;msg.info.width=msg.info.height=grid.width
            msg.info.origin.position.x=msg.info.origin.position.y=grid.origin;msg.info.origin.orientation.w=1.
            msg.data=grid.cost.ravel().tolist();fingerprints.append(fingerprint(msg));publisher.publish(msg)
        self.fingerprints=fingerprints

    def on_reset(self,request,response):
        """只重置观测地图，保留连续LIO状态与坐标修正；代次变化使旧任务结束。"""
        old=self.evidence;new=TerrainEvidence(classified=self.classified)
        new.transform=old.transform.copy();new.revision=old.revision;new.generation=old.generation+1
        self.evidence=new;self.minimum_stamp=self.get_clock().now().nanoseconds*1e-9
        self.publish_maps();self.tick();response.success=True;response.message='地图已清空，等待新观测；LIO连续状态保留'
        return response

    def publish_transform(self):
        x,y,yaw=self.evidence.transform
        tf=TransformStamped();tf.header.frame_id='lio_map';tf.child_frame_id='lio_odom'
        tf.header.stamp=self.get_clock().now().to_msg()
        tf.transform.translation.x=float(x);tf.transform.translation.y=float(y)
        tf.transform.rotation.z=math.sin(yaw/2);tf.transform.rotation.w=math.cos(yaw/2)
        self.tf.sendTransform(tf)

    def save_maps(self):
        if not self.evidence.groups:return None
        self.folder.mkdir(parents=True,exist_ok=True)
        for name,grid in [('global',self.evidence.global_grid),('local',self.evidence.local_grid)]:
            temp=self.folder/(name+'.tmp')
            with temp.open('wb') as stream:
                np.savez_compressed(stream,cost=grid.cost,observed_at=grid.seen,resolution=grid.resolution,
                    origin=[grid.origin,grid.origin],frame_id='lio_map',source=self.source,
                    map_to_odom=self.evidence.transform,correction_revision=self.evidence.revision,
                    reset_generation=self.evidence.generation)
            temp.replace(self.folder/(name+'.npz'))
        # 证据归档保持源坐标，修正后仍可审计原始测量；本阶段不自动跨会话加载。
        groups=list(self.evidence.groups.values());temp=self.folder/'evidence.tmp'
        with temp.open('wb') as stream:
            np.savez_compressed(stream,points=np.concatenate([p for p,_ in groups]),
                stamps=np.concatenate([np.full(len(p),s) for p,s in groups]),frame_id='lio_odom',source=self.source)
        temp.replace(self.folder/'evidence.npz')
        self.storage_error='';self.last_save=time.monotonic()
        return self.folder

    def on_save(self,request,response):
        try:
            folder=self.save_maps();response.success=folder is not None
            response.message=str(folder) if folder else '暂无有效观测'
        except OSError as exc:response.success=False;response.message=str(exc);self.storage_error=str(exc)
        return response

    def tick(self):
        wall=time.monotonic();now=self.get_clock().now().nanoseconds*1e-9
        if self.last_clock is not None and now<self.last_clock-1e-6:self.reset_observations()
        self.last_clock=now
        if self.evidence.local_grid.expire(now):self.publish_maps()
        self.publish_transform()
        if self.evidence.groups and wall-self.last_save>=5.:
            try:self.save_maps()
            except OSError as exc:self.storage_error=str(exc);self.last_save=wall
        grid=self.evidence.global_grid
        ready=bool(self.source and self.evidence.groups)
        self.status.publish(String(data=json.dumps(dict(schema_version=1,ready=ready,source=self.source,
            frame_id='lio_map',source_frame='lio_odom',revision=self.updates,reset_generation=self.evidence.generation,
            correction_revision=self.evidence.revision,map_to_odom=self.evidence.transform.tolist(),
            map_fingerprints=self.fingerprints,
            bounds_m=[grid.origin,grid.origin,grid.origin+grid.size,grid.origin+grid.size],
            width=grid.width,height=grid.width,resolution=grid.resolution,
            known_cells=int((grid.cost>=0).sum()),local_known_cells=int((self.evidence.local_grid.cost>=0).sum()),
            evidence_cells=len(self.evidence.groups),terrain_rejections=self.rejected,last_error=self.last_error,
            terrain_sim_age_sec=now-self.evidence.last_stamp,last_correction=self.last_correction,
            storage_error=self.storage_error,navigation_input=os.environ.get('FOREST_NAV_LOCALIZATION')=='lio'),ensure_ascii=False)))


def main():
    rclpy.init();node=LioCostmap()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        # launch同时关闭多个进程时可能再次发SIGINT；第一次已进入退出流程，避免打断原子归档。
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        try:node.save_maps()
        except OSError as exc:node.get_logger().error(str(exc))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
