"""独立雷达帧 → ROS观测；状态心跳与新测量分开，旧点云绝不重复发布续命。"""
import json
import math
import os
import time
from pathlib import Path
from zipfile import BadZipFile
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header,String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rosgraph_msgs.msg import Clock as ClockMessage
from .mapping_diagnostics import DiagnosticLog
from .observation_health import ProgressClock,scan_period
from .imu_bridge import InertialSync
from .sensor_contract import imu_pose_from_lidar


class LidarBridge(Node):
    def __init__(self):
        super().__init__('forest_lidar_bridge')
        self.file=Path(os.environ['FOREST_LIDAR_FRAME_FILE'])
        self.progress=ProgressClock(self.file.with_name('lidar_progress.json'))
        self.max_scan_sim_age=2*scan_period()
        self.previous=None;self.last_valid=None;self.count=0;self.clock_identity=None
        self.diagnostic_log=DiagnosticLog('lidar_bridge');self.last_failure=None;self.rejected_frames=0
        self.diag=self.create_publisher(String,'/lidar/diagnostics',10)
        self.last_tick_wall=None;self.last_published_wall=None
        self.pub=self.create_publisher(PointCloud2,'/lidar/points',5)
        self.registered=self.create_publisher(PointCloud2,'/lidar/points_map',5)
        self.odom=self.create_publisher(Odometry,'/lidar/ground_truth_odometry',5)
        self.tf=TransformBroadcaster(self)
        self.status=self.create_publisher(String,'/lidar/status',10)
        # 独立传感器模式仍由本节点提供/clock，但按每一步进度而非雷达采样推进。
        self.clock=self.create_publisher(ClockMessage,'/clock',10) if os.environ.get('FOREST_LIDAR_PUBLISH_CLOCK')=='1' else None
        self.sensors=InertialSync(self,self.file.with_name('imu_buffer.npz'),self.stamp_message)
        self.timer=self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    @staticmethod
    def stamp_message(stamp):
        from builtin_interfaces.msg import Time
        total=round(stamp*1e9);sec,ns=divmod(total,1000000000)
        return Time(sec=sec,nanosec=ns)

    def publish_frame(self,points,world,pose,stamp,rings=None,point_time=None):
        """只有新且有效的帧进入这个方法，CMU不会收到伪造的新观测。"""
        t=self.stamp_message(stamp)
        fields=[PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1)
                for i,n in enumerate(('x','y','z','intensity'))]
        def message(values,frame):
            return point_cloud2.create_cloud(Header(frame_id=frame,stamp=t),fields,
                np.c_[values,np.zeros(len(values),dtype=np.float32)].tolist())
        raw=message(points,'lidar_link');synchronized=raw
        if rings is not None:
            if rings.shape!=(len(points),) or point_time.shape!=(len(points),) or not np.isfinite(point_time).all():
                raise ValueError('雷达ring/time结构无效')
            raw_fields=fields+[PointField(name='ring',offset=16,datatype=PointField.UINT16,count=1),
                PointField(name='time',offset=20,datatype=PointField.FLOAT32,count=1)]
            records=np.zeros(len(points),dtype=point_cloud2.dtype_from_fields(raw_fields))
            for i,name in enumerate(('x','y','z')):records[name]=points[:,i]
            records['ring']=rings;records['time']=point_time
            synchronized=point_cloud2.create_cloud(Header(frame_id='lidar_link',stamp=t),raw_fields,records)
        self.pub.publish(raw)
        if os.environ.get('FOREST_NAV_LOCALIZATION')!='lio':self.registered.publish(message(world,'map'))
        if rings is not None:self.sensors.queue_cloud(synchronized,stamp)
        odom=Odometry();odom.header=Header(frame_id='map',stamp=t);odom.child_frame_id='lidar_link'
        odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=map(float,pose[:3])
        q=odom.pose.pose.orientation;q.x,q.y,q.z,q.w=map(float,pose[3:])
        self.odom.publish(odom)
        # 真值里程计仅保留给独立评估；LIO导航下不再发布真值传感器TF。
        if os.environ.get('FOREST_NAV_LOCALIZATION')=='lio':return
        # 保留原真值导航：map→imu_link为完整6DoF真值，imu_link→lidar_link是固定安装。
        # 不能把imu_link静态挂在现有平面base_link下，那会丢失人形机身滚转/俯仰。
        imu_pose=imu_pose_from_lidar(pose)
        tf=TransformStamped();tf.header=odom.header;tf.child_frame_id='imu_link'
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,imu_pose[:3])
        tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,imu_pose[3:])
        self.tf.sendTransform(tf)

    def tick(self):
        started=time.monotonic()
        progressing=self.progress.poll(started)
        identity=(self.progress.session,self.progress.epoch)
        if identity!=self.clock_identity:
            self.previous=None;self.last_valid=None;self.clock_identity=identity
        diagnostic=dict(wall_monotonic=started,event='poll',
            timer_lag_sec=0. if self.last_tick_wall is None else max(0.,started-self.last_tick_wall-.05),
            published_wall_gap_sec=None if self.last_published_wall is None else started-self.last_published_wall,
            progress_ready=progressing,progress_reason=self.progress.reason,sim_time=self.progress.stamp)
        self.last_tick_wall=started
        self.sensors.poll(self.progress,progressing,started)
        try:
            with np.load(self.file,allow_pickle=False) as frame:
                sent=float(frame['sent_monotonic']);stamp=float(frame['stamp'])
                key=(str(frame['session']),int(frame['sequence']))
                diagnostic.update(sequence=key[1],session=key[0],stamp=stamp,frame_wall_age_sec=started-sent)
                if not math.isfinite(sent):raise ValueError('雷达帧发送时间无效')
                if self.progress.rewind_sent is not None and sent<self.progress.rewind_sent:
                    raise ValueError('拒收仿真时钟回退前的雷达帧')
                if not math.isfinite(stamp) or stamp<=0:raise ValueError('雷达帧时间戳无效')
                if key[0]!=self.progress.session:raise ValueError('雷达帧与仿真步进会话不一致')
                age=self.progress.stamp-stamp
                if key==self.previous:
                    diagnostic['event']='duplicate'
                else:
                    if self.previous and (key[1]<=self.previous[1] or stamp<=self.last_valid['stamp']):
                        raise ValueError('雷达帧重复或乱序')
                    points=frame['points'].copy();world=frame['world_points'].copy()
                    pose=frame['sensor_pose'].copy();body=frame['body_pose'].copy()
                    if points.ndim!=2 or points.shape[1]!=3 or world.shape!=points.shape or pose.shape!=(7,) or body.shape!=(7,):
                        raise ValueError('雷达帧结构无效')
                    if not all(np.isfinite(a).all() for a in (points,world,pose,body)):
                        raise ValueError('雷达帧数值无效')
                    rings=frame['rings'].copy() if 'rings' in frame else None
                    point_time=frame['point_time'].copy() if rings is not None else None
                    self.publish_frame(points,world,pose,stamp,rings,point_time)
                    self.previous=key;self.count+=1;self.last_published_wall=time.monotonic()
                    self.last_valid=dict(session=key[0],sequence=key[1],stamp=stamp,points=len(points))
                    diagnostic['event']='published';self.last_failure=None
        except (OSError,ValueError,TypeError,KeyError,EOFError,BadZipFile) as exc:
            diagnostic.update(event='rejected',reason=str(exc))
            self.rejected_frames+=1
            if str(exc)!=self.last_failure:self.diagnostic_log.write('frame_rejected',details=diagnostic)
            self.last_failure=str(exc)
        # 单帧读坏时保留上一有效测量的原始时间；绝不因错误或心跳延长其有效期。
        valid=self.last_valid
        age=None if valid is None or self.progress.stamp is None else self.progress.stamp-valid['stamp']
        ready=bool(valid is not None and valid['session']==self.progress.session)
        reason='' if ready else self.progress.reason or self.last_failure or '等待有效雷达观测'
        diagnostic.update(processing_sec=time.monotonic()-started,scan_sim_age_sec=age)
        self.status.publish(String(data=json.dumps(dict(ready=ready,reason=reason,
            session=None if valid is None else valid['session'],sequence=None if valid is None else valid['sequence'],
            stamp=None if valid is None else valid['stamp'],points=0 if valid is None else valid['points'],
            clock_epoch=self.progress.epoch,published_frames=self.count,rejected_frames=self.rejected_frames,
            heartbeat=True,max_scan_sim_age_sec=self.max_scan_sim_age,pose_source='isaac_ground_truth',diagnostics=diagnostic),ensure_ascii=False)))
        if self.clock and progressing:self.clock.publish(ClockMessage(clock=self.stamp_message(self.progress.stamp)))
        self.diag.publish(String(data=json.dumps(diagnostic,ensure_ascii=False)))
        self.sensors.flush(self.progress,progressing)


def main():
    rclpy.init();node=LidarBridge()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
