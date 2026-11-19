"""ROS侧原始IMU与激光配对；同一节点先发IMU，再发已被IMU覆盖的点云。"""
from collections import deque
import json
import math
from zipfile import BadZipFile
import numpy as np
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu,PointCloud2
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from .sensor_contract import imu_settings,extrinsics,validate_imu_packet
from .observation_health import scan_period


class InertialSync:
    def __init__(self,node,path,stamp_message):
        self.node=node;self.path=path;self.stamp_message=stamp_message;self.settings=imu_settings()
        self.period_ns=round(1e9/self.settings['frequency_hz']);self.identity=None
        self.max_cloud_sim_age=2*scan_period()
        self.pub=node.create_publisher(Imu,'/imu/data_raw',1000)
        self.cloud_pub=node.create_publisher(PointCloud2,'/lidar/points_synced',10)
        self.status_pub=node.create_publisher(String,'/sensors/sync_status',10)
        self.static=StaticTransformBroadcaster(node)
        t,q=extrinsics();tf=TransformStamped();tf.header.frame_id='imu_link';tf.child_frame_id='lidar_link'
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z=map(float,t)
        tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,q)
        self.static.sendTransform(tf)
        self.reset(None)

    def reset(self,identity):
        self.identity=identity;self.last_sequence=None;self.last_ns=None;self.first_ns=None
        self.count=0;self.pending=deque(maxlen=4);self.synced_count=0;self.last_cloud_ns=None
        self.fault='';self.last_error='等待IMU';self.gap_count=0;self.dropped_clouds=0;self.last_read_ns=None

    def poll(self,progress,progressing,now):
        identity=(progress.session,progress.epoch)
        if identity!=self.identity:self.reset(identity)
        try:
            mtime=self.path.stat().st_mtime_ns
            with np.load(self.path,allow_pickle=False) as packet:
                if str(packet['session'])!=progress.session:raise ValueError('IMU会话与激光时钟不一致')
                sent=float(packet['sent_monotonic']);period=int(packet['period_ns'])
                if not math.isfinite(sent) or sent>now:raise ValueError('IMU发送时间无效')
                if progress.rewind_sent is not None and sent<progress.rewind_sent:raise ValueError('IMU缓冲来自时钟回退前')
                stamps=packet['stamps_ns'].copy();seq=packet['sequences'].copy();values=packet['measurements'].copy()
            if period<=0:raise ValueError('IMU实际采样周期必须为正')
            self.period_ns=period
            validate_imu_packet(stamps,seq,values,period)
            if int(stamps[-1])>round(progress.stamp*1e9)+1:
                # 进度文件与IMU文件分别原子更新；并发读取时等下一次重新读进度，
                # 不将这个暂时先后关系当成实际样本乱序。
                raise ValueError('等待IMU对应的仿真时钟')
            if self.last_sequence is not None and int(seq[-1])<self.last_sequence:
                raise ValueError('拒收回退的IMU缓冲')
            fresh=np.ones(len(seq),dtype=bool) if self.last_sequence is None else seq>self.last_sequence
            for stamp,sequence,value in zip(stamps[fresh],seq[fresh],values[fresh]):
                stamp=int(stamp);sequence=int(sequence)
                if self.last_sequence is not None and (sequence!=self.last_sequence+1 or stamp-self.last_ns!=period):
                    self.gap_count+=1;self.pending.clear();self.last_cloud_ns=None;self.first_ns=None
                message=Imu();message.header.frame_id='imu_link'
                # 保持整数纳秒，不经过浮点秒往返，避免长时间运行时丢失低位时间。
                sec,ns=divmod(stamp,1000000000)
                message.header.stamp=Time(sec=sec,nanosec=ns)
                # 这里是原始惯性测量，不伪造姿态融合结果。未知姿态按ROS约定标为-1。
                message.orientation.w=1.;message.orientation_covariance[0]=-1.
                message.angular_velocity.x,message.angular_velocity.y,message.angular_velocity.z=map(float,value[:3])
                message.linear_acceleration.x,message.linear_acceleration.y,message.linear_acceleration.z=map(float,value[3:])
                message.angular_velocity_covariance=self.settings['angular_velocity_covariance']
                message.linear_acceleration_covariance=self.settings['linear_acceleration_covariance']
                self.pub.publish(message)
                if self.first_ns is None:self.first_ns=stamp
                self.last_ns,self.last_sequence=stamp,sequence;self.count+=1
            self.last_read_ns=mtime;self.last_error=''
        except (OSError,ValueError,TypeError,KeyError,EOFError,BadZipFile) as exc:
            self.last_error=str(exc)  # 不刷新测量时间，旧测量会按原期限失效。

    def queue_cloud(self,message,stamp):
        if len(self.pending)==self.pending.maxlen:self.dropped_clouds+=1
        stamp_ns=message.header.stamp.sec*1000000000+message.header.stamp.nanosec
        if (self.last_cloud_ns is not None and stamp_ns<=self.last_cloud_ns) or (self.pending and stamp_ns<=self.pending[-1][0]):
            self.dropped_clouds+=1;return
        self.pending.append((stamp_ns,message))

    def flush(self,progress,progressing):
        while self.pending and self.last_ns is not None:
            stamp,message=self.pending[0]
            if self.last_ns<stamp:break
            begin=self.last_cloud_ns if self.last_cloud_ns is not None else stamp
            if self.first_ns>begin:
                self.pending.popleft();self.dropped_clouds+=1;continue
            # 此前已确认整段IMU连续，且包含本帧点云的时间终点，才发布同步入口。
            self.cloud_pub.publish(message);self.last_cloud_ns=stamp;self.synced_count+=1;self.pending.popleft()
        imu_age=None if self.last_ns is None or progress.stamp is None else progress.stamp-self.last_ns/1e9
        cloud_age=None if self.last_cloud_ns is None or progress.stamp is None else progress.stamp-self.last_cloud_ns/1e9
        ready=bool(self.synced_count>0)
        status=dict(schema_version=1,ready=ready,session=progress.session,clock_epoch=progress.epoch,
            reason_code=self.fault or ('' if ready else 'waiting_synchronized_measurements'),last_error=self.last_error,
            imu_topic='/imu/data_raw',lidar_topic='/lidar/points_synced',imu_frame='imu_link',lidar_frame='lidar_link',
            imu_frequency_hz=1e9/self.period_ns,imu_published=self.count,
            imu_last_stamp_ns=self.last_ns,imu_sim_age_sec=imu_age,last_synced_lidar_stamp_ns=self.last_cloud_ns,
            synced_clouds=self.synced_count,lidar_sim_age_sec=cloud_age,imu_gaps=self.gap_count,dropped_clouds=self.dropped_clouds,
            clock_source='same_physics_steps',orientation_available=False,
            acceleration_convention='proper_acceleration_with_gravity',scan_model='instantaneous_all_point_time_zero')
        self.status_pub.publish(String(data=json.dumps(status,ensure_ascii=False)))
