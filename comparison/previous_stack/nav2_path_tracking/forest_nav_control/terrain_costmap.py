"""CMU地形点云 → 标准代价栅格；不读取离线地图、USD或相机。

CMU terrain_map的intensity是点相对估计地面的高度差（米），不是0—100代价。
这里按当前G1暂定几何阈值编码；它不估计松土承载力，也不证明策略必然能通过。
只更新实际收到地形点的单元；邻域、机器人脚下、未观测区都不自动填自由。
"""
import json
import math
import os
import time
import hashlib
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .mapping_diagnostics import health_checks,DiagnosticLog
from .observation_health import scan_period


class TerrainGrid:
    """固定地图坐标下累积已观测格子；扩展只增加未知边框，不移动历史世界坐标。"""
    def __init__(self,resolution=.2,size=32.,ttl=None,expand=False):
        self.resolution=float(resolution);self.size=float(size)
        self.ttl=None if ttl is None else float(ttl)
        if not all(math.isfinite(v) and v>0 for v in (resolution,size)):
            raise ValueError('栅格及阈值参数必须为正数')
        if self.ttl is not None and (not math.isfinite(self.ttl) or self.ttl<=0):
            raise ValueError('观测有效期必须为正数或None（长期保留）')
        self.initial_size=float(size);self.expand=bool(expand)
        self.reset()

    def reset(self):
        self.size=self.initial_size;self.width=int(round(self.size/self.resolution));self.origin=-self.width*self.resolution/2
        self.cost=np.full((self.width,self.width),-1,dtype=np.int8)
        self.seen=np.full(self.cost.shape,-np.inf)
        self.last_stamp=0.

    def ensure_bounds(self,xy):
        """对称按8米左右的整格边框扩展；原格子只做整数索引搬运，无插值/缩放。"""
        if not self.expand or not len(xy):return False
        xy=np.asarray(xy)
        if not np.isfinite(xy).all():raise ValueError('扩展范围包含非有限坐标')
        half=self.width*self.resolution/2
        required=max(float(np.max(xy))+.5*self.resolution,float(-np.min(xy)))
        if required<half:return False
        # 分配前检查，避免一帧错误定位触发超大内存申请。超限拒收，绝不裁掉后冒充完整地图。
        step=max(1,int(round(4./self.resolution)))
        added=int(math.ceil((required-half+2.)/self.resolution/step))*step
        width=self.width+2*added
        cost=np.full((width,width),-1,dtype=np.int8);seen=np.full(cost.shape,-np.inf)
        cost[added:added+self.width,added:added+self.width]=self.cost
        seen[added:added+self.width,added:added+self.width]=self.seen
        self.cost,self.seen=cost,seen;self.width=width;self.origin-=added*self.resolution
        self.size=width*self.resolution
        return True

    def expire(self,stamp):
        """过期只恢复未知，绝不能把未再次观测的障碍清成自由。"""
        if self.ttl is None:return False
        expired=(self.cost>=0)&((stamp-self.seen)>self.ttl)
        if not expired.any():return False
        self.cost[expired]=-1;self.seen[expired]=-np.inf
        return True

    def update(self,points,stamp):
        if not math.isfinite(stamp) or stamp<=0:raise ValueError('地形点云时间戳无效')
        # 消息时间倒退仅表示重复/乱序；清图只能由显式重置、会话或时钟事件触发。
        if stamp<=self.last_stamp:return False
        points=np.asarray(points,dtype=np.float64).reshape(-1,4)
        valid=np.isfinite(points).all(axis=1)&(points[:,3]>=0)
        points=points[valid]
        self.ensure_bounds(points[:,:2])
        index=np.floor((points[:,:2]-self.origin)/self.resolution).astype(int)
        valid=(index>=0).all(axis=1)&(index<self.width).all(axis=1)
        points,index=points[valid],index[valid]
        # 点云中没有点的单元没有测量证据；不能用插值或清空整片视野将它变成可走。
        self.expire(stamp)
        if len(points)==0:return False
        values=np.clip(np.rint(points[:,3]*100.),0,99).astype(np.int16)
        flat=index[:,1]*self.width+index[:,0]
        reduced=np.full(self.cost.size,-1,dtype=np.int16)
        np.maximum.at(reduced,flat,values)  # 同格树干和地面同时存在时，较高代价优先。
        observed=reduced>=0
        self.cost.ravel()[observed]=reduced[observed].astype(np.int8)
        self.seen.ravel()[observed]=stamp
        self.last_stamp=stamp
        return True


class TerrainCostmap(Node):
    def __init__(self):
        super().__init__('cmu_terrain_costmap',parameter_overrides=[Parameter('use_sim_time',value=True)])
        # 全局保存已探索区域；局部仅保留近期观测。两张图共享坐标和代价编码。
        self.grid=TerrainGrid(ttl=None,expand=True)
        self.local_grid=TerrainGrid(ttl=None,expand=True)
        self.archive=Path(os.environ.get('FOREST_NAV_MAP_ARCHIVE',
            str(Path(os.environ.get('FOREST_NAV_READINESS_FILE',
                str(Path(__file__).resolve().parents[2]/'forest_lidar_perception/results/live/readiness.json'))).parent/'maps')))
        self.saved_revision=-1;self.last_save_wall=0.;self.minimum_stamp=0.;self.storage_error=''

        self.max_terrain_sim_age=3*scan_period()
        self.last_clock_time=None;self.source_epoch=None;self.reset_generation=0
        self.lidar_received=None;self.terrain_received=None
        self.lidar_ready=False;self.session=None;self.revision=0;self.error='等待雷达与CMU地形点云'
        self.diagnostic_log=DiagnosticLog('terrain_costmap')
        self.last_health_signature=None;self.last_tick_wall=None;self.last_timer_lag=0.
        self.last_terrain_input=None;self.last_rejection=None
        self.terrain_inputs=0;self.terrain_rejections=0;self.terrain_updates=0;self.terrain_duplicates=0
        self.last_update_sec=0.;self.last_save_sec=0.;self.last_publish_sec=0.
        self.initialized=False
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.pub=self.create_publisher(OccupancyGrid,'/traversability_map',qos)
        self.local_pub=self.create_publisher(OccupancyGrid,'/traversability_map_local',qos)
        self.save_service=self.create_service(Trigger,'/mapping/save',self.save_service_callback)
        self.reset_service=self.create_service(Trigger,'/mapping/reset',self.reset_service_callback)
        self.status=self.create_publisher(String,'/navigation/mapping_status',10)
        self.sub=self.create_subscription(PointCloud2,'/terrain_map',self.on_terrain,2)
        self.lidar_sub=self.create_subscription(String,'/lidar/status',self.on_lidar,10)
        self.timer=self.create_timer(.2,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_lidar(self,msg):
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict) or not isinstance(data.get('ready'),bool):raise ValueError('雷达状态格式无效')
            if not data.get('ready'):
                self.lidar_ready=False;self.error=data.get('reason','雷达无效');return
            session=data.get('session')
            if not isinstance(session,str) or not session:raise ValueError('雷达会话标识无效')
            epoch=data.get('clock_epoch',0)
            if not isinstance(epoch,int) or epoch<0:raise ValueError('雷达时钟代号无效')
            if session!=self.session or epoch!=self.source_epoch:
                if self.session is not None:
                    try:self.save_maps()
                    except OSError as exc:self.storage_error=str(exc)
                self.grid.reset();self.local_grid.reset();self.terrain_received=None;self.session=session;self.source_epoch=epoch
                self.minimum_stamp=0.;self.revision+=1;self.saved_revision=-1;self.reset_generation+=1
                self.publish_map(self.get_clock().now().to_msg())
            self.lidar_received=time.monotonic();self.lidar_ready=True
        except (ValueError,TypeError):pass  # 拒收异常状态，原有效心跳按原时间自然过期。

    def on_terrain(self,msg):
        started=time.monotonic()
        self.terrain_inputs+=1
        self.last_terrain_input=dict(wall_monotonic=started,frame_id=msg.header.frame_id,
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
            sim_time=self.get_clock().now().nanoseconds/1e9,points=msg.width*msg.height)
        try:
            if msg.header.frame_id!='map':raise ValueError('CMU地形点云必须位于map坐标')
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            age=self.observe_clock()-stamp
            if stamp<self.minimum_stamp:return
            points=point_cloud2.read_points_numpy(msg,field_names=('x','y','z','intensity'),skip_nans=True)
            updated=self.grid.update(points,stamp)
            local_updated=self.local_grid.update(points,stamp)
            if updated or local_updated:
                self.revision+=1;self.terrain_received=time.monotonic();self.error='';self.terrain_updates+=1
                self.publish_map(msg.header.stamp)
            else:self.terrain_duplicates+=1
        except (ValueError,TypeError,KeyError,AssertionError) as exc:
            # 拒收坏帧不刷新上一有效测量，也不撤销它尚未耗尽的有效期。
            self.error=str(exc)
            self.terrain_rejections+=1
            self.last_rejection=dict(self.last_terrain_input,reason=str(exc))
            self.diagnostic_log.write('terrain_rejected',**self.last_rejection)
        finally:self.last_update_sec=time.monotonic()-started

    def observe_clock(self):
        current=self.get_clock().now().nanoseconds/1e9
        if self.last_clock_time is not None and current<self.last_clock_time-1e-6:
            # 依据真实/clock回退清图，不再依据一帧消息的时间戳猜测时钟回退。
            self.grid.reset();self.local_grid.reset();self.terrain_received=None
            self.minimum_stamp=current;self.revision+=1;self.reset_generation+=1;self.error='仿真时钟回退，等待新观测'
            self.diagnostic_log.write('clock_rewound',previous=self.last_clock_time,current=current)
            self.publish_map(self.get_clock().now().to_msg())
        self.last_clock_time=current
        return current

    def publish_map(self,stamp):
        started=time.monotonic()
        for grid,publisher in ((self.grid,self.pub),(self.local_grid,self.local_pub)):
            msg=OccupancyGrid();msg.header.frame_id='map';msg.header.stamp=stamp
            msg.info.resolution=grid.resolution;msg.info.width=msg.info.height=grid.width
            msg.info.origin.position.x=msg.info.origin.position.y=grid.origin
            msg.info.origin.orientation.w=1.
            msg.data=grid.cost.ravel().tolist();publisher.publish(msg)
        self.last_publish_sec=time.monotonic()-started

    def save_maps(self):
        """按定位会话分目录原子保存；不自动加载另一会话，避免坐标不一致。"""
        if self.session is None:return None
        started=time.monotonic()
        folder=self.archive/hashlib.sha256(self.session.encode()).hexdigest()[:16]
        folder.mkdir(parents=True,exist_ok=True)
        for name,grid in (('global',self.grid),('local',self.local_grid)):
            path=folder/(name+'.npz');temporary=path.with_suffix('.tmp')
            with temporary.open('wb') as stream:
                np.savez_compressed(stream,cost=grid.cost,observed_at=grid.seen,
                    resolution=grid.resolution,origin=[grid.origin,grid.origin],frame_id='map',
                    session=self.session,revision=self.revision)
            temporary.replace(path)
        metadata=dict(schema_version=1,session=self.session,revision=self.revision,
            global_topic='/traversability_map',local_topic='/traversability_map_local',
            global_retention='until_session_reset_or_reobserved',local_ttl_sec=self.local_grid.ttl,
            auto_reload=False,saved_wall_time=time.time())
        temp=folder/'metadata.tmp';temp.write_text(json.dumps(metadata,ensure_ascii=False,indent=2));temp.replace(folder/'metadata.json')
        self.saved_revision=self.revision;self.last_save_wall=time.monotonic();self.storage_error=''
        self.last_save_sec=time.monotonic()-started
        return folder

    def save_service_callback(self,request,response):
        try:
            folder=self.save_maps();response.success=folder is not None
            response.message=str(folder) if folder else '没有有效地图会话'
        except OSError as exc:
            response.success=False;response.message=str(exc);self.storage_error=str(exc)
        return response

    def reset_service_callback(self,request,response):
        # 先发布全未知并撤销地图ready，监视器会终止旧任务；新观测才可重新放行。
        self.grid.reset();self.local_grid.reset();self.terrain_received=None;self.revision+=1;self.reset_generation+=1
        self.minimum_stamp=self.get_clock().now().nanoseconds/1e9
        self.error='地图已重置，等待新观测';self.publish_map(self.get_clock().now().to_msg())
        self.tick();response.success=True;response.message=self.error
        return response

    def tick(self):
        now=time.monotonic()
        self.last_timer_lag=0. if self.last_tick_wall is None else max(0.,now-self.last_tick_wall-.2)
        self.last_tick_wall=now
        self.observe_clock()
        stamp=self.get_clock().now()
        if self.local_grid.expire(stamp.nanoseconds/1e9):
            self.revision+=1;self.publish_map(stamp.to_msg())
        if self.session is not None and self.revision!=self.saved_revision and now-self.last_save_wall>=5.:
            try:self.save_maps()
            except OSError as exc:self.storage_error=str(exc);self.last_save_wall=now
        # 发布初始全未知地图，使Nav2可完成地图订阅；它不代表已可控制或已观测到地面。
        if not self.initialized:
            self.publish_map(self.get_clock().now().to_msg());self.initialized=True
        checks=health_checks(self.lidar_ready,self.lidar_received,self.terrain_received,
            self.grid.last_stamp,stamp.nanoseconds/1e9,now,self.max_terrain_sim_age)
        ready=bool(self.grid.last_stamp>0)
        diagnostic=dict(checks=checks,failed_checks=[name for name,ok in checks.items() if not ok],
            sim_time=stamp.nanoseconds/1e9,last_terrain_stamp=self.grid.last_stamp,
            last_terrain_input=self.last_terrain_input,last_rejection=self.last_rejection,
            terrain_inputs=self.terrain_inputs,terrain_updates=self.terrain_updates,
            terrain_rejections=self.terrain_rejections,terrain_duplicates=self.terrain_duplicates,
            update_sec=self.last_update_sec,save_sec=self.last_save_sec,publish_sec=self.last_publish_sec,
            timer_lag_sec=self.last_timer_lag)
        signature=tuple(diagnostic['failed_checks'])
        if signature!=self.last_health_signature:
            self.diagnostic_log.write('health_transition',ready=ready,**diagnostic)
            self.last_health_signature=signature
        self.status.publish(String(data=json.dumps(dict(ready=ready,source='cmu_lidar',
            schema_version=1,frame_id='map',diagnostics=diagnostic,
            max_terrain_sim_age_sec=self.max_terrain_sim_age,clock_epoch=self.source_epoch,reset_generation=self.reset_generation,
            reason='' if ready else self.error or '雷达/地形地图更新超时',
            reason_code='' if ready else ('waiting_observation' if self.terrain_received is None else 'observation_timeout'),revision=self.revision,
            lidar_wall_age_sec=None if self.lidar_received is None else now-self.lidar_received,
            terrain_wall_age_sec=None if self.terrain_received is None else now-self.terrain_received,
            terrain_sim_age_sec=stamp.nanoseconds/1e9-self.grid.last_stamp,
            session=self.session,known_cells=int((self.grid.cost>=0).sum()),
            bounds_m=[self.grid.origin,self.grid.origin,self.grid.origin+self.grid.size,self.grid.origin+self.grid.size],
            map_width=self.grid.width,map_height=self.grid.width,map_resolution=self.grid.resolution,
            blocked_cells=int((self.grid.cost==100).sum()),unknown_cells=int((self.grid.cost<0).sum()),
            global_topic='/traversability_map',local_topic='/traversability_map_local',
            global_retention='persistent_in_session',local_ttl_sec=self.local_grid.ttl,
            local_known_cells=int((self.local_grid.cost>=0).sum()),storage_error=self.storage_error,
            cost_source='cmu_relative_height_cm_soft_cost'),ensure_ascii=False)))


def main():
    rclpy.init();node=TerrainCostmap()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if rclpy.ok():
            node.status.publish(String(data=json.dumps(dict(ready=False,source='cmu_lidar',reason='地图节点已关闭'))))
        try:node.save_maps()
        except OSError as exc:node.get_logger().error('地图存档失败：'+str(exc))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
