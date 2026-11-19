"""仅在Isaac进程加载：原生IMU + 公开物理步记录回调，不改IsaacLab核心。"""
from collections import deque
import os
import time
import numpy as np
import torch
from isaaclab.managers import RecorderTerm,RecorderTermCfg
from isaaclab.managers.recorder_manager import DatasetExportMode
from isaaclab.sensors import ImuCfg
from .sensor_contract import imu_settings


def configure_imu(cfg):
    settings=imu_settings();period=1./settings['frequency_hz'];dt=float(cfg.sim.dt)
    # 与雷达挂在同一个torso刚体。原生PhysX IMU包含安装点杠杆臂补偿和重力。
    cfg.scene.navigation_imu=ImuCfg(prim_path=cfg.scene.camera.prim_path,update_period=dt,
        offset=ImuCfg.OffsetCfg(pos=tuple(settings['offset_torso_m']),rot=tuple(settings['rotation_xyzw'])))
    active=any(isinstance(value,RecorderTermCfg) for value in vars(cfg.recorders).values())
    if not active:cfg.recorders.dataset_export_mode=DatasetExportMode.EXPORT_NONE
    cfg.recorders.navigation_imu=RecorderTermCfg(class_type=ImuPhysicsCapture)


class ImuPhysicsCapture(RecorderTerm):
    def record_post_physics_decimation_step(self):
        sink=getattr(self._env,'_forest_imu_sink',None)
        if sink is not None:
            step=int(self._env._sim_step_counter)-sink.base_step
            # 第一个物理步前传感器时间为0，原生IMU有初始化保护，丢弃该暖机样本。
            if step>1 and step%sink.stride==0:
                sensor=self._env.scene['navigation_imu']
                # 回调发生在物理步之后、scene.update之前。dt=0只强制读取本步求解器
                # 数据，不重复推进传感器时钟；场景仍负责正常的时间更新。
                sensor.update(0.,force_recompute=True)
                data=sensor.data
                value=torch.cat((data.ang_vel_b.torch[0],data.lin_acc_b.torch[0])).clone()
                sink.capture(step,value)
        return None,None  # 不写训练数据集，也不影响动作/奖励/策略。


class ImuExporter:
    def __init__(self,env,session,directory):
        self.env=env;self.session=session;self.settings=imu_settings()
        self.base_step=int(env._sim_step_counter);self.dt_ns=round(float(env.physics_dt)*1e9)
        self.stride=max(1,round(1e9/self.settings['frequency_hz']/self.dt_ns))
        if getattr(env,'_physics_handles_decimation',False):
            batch=int(env.cfg.decimation)
            self.stride=batch*max(1,round(self.stride/batch))
        self.period_ns=self.stride*self.dt_ns
        self.pending=[];self.sequence=0
        self.history=deque(maxlen=round(self.settings['history_sec']*self.settings['frequency_hz']))
        self.destination=directory/'imu_buffer.npz'
        env._forest_imu_sink=self

    def capture(self,step,value):
        self.sequence+=1
        stamp=1_000_000_000+step*self.dt_ns
        self.pending.append((stamp,self.sequence,value))

    def flush(self):
        if not self.pending:return
        # 子步先在GPU保存6个数，控制步末统一搬到CPU；不能把50Hz末状态插值成200Hz。
        values=torch.stack([p[2] for p in self.pending]).detach().cpu().numpy()
        for (stamp,sequence,_),value in zip(self.pending,values):self.history.append((stamp,sequence,value))
        self.pending.clear()
        stamps=np.asarray([p[0] for p in self.history],dtype=np.int64)
        seq=np.asarray([p[1] for p in self.history],dtype=np.int64)
        measurements=np.stack([p[2] for p in self.history]).astype(np.float64)
        temporary=self.destination.with_suffix('.tmp')
        with temporary.open('wb') as stream:
            # 保留3仿真秒滚动历史，ROS轮询即使跨过多个控制步，也不会只读到最后4个样本。
            np.savez(stream,stamps_ns=stamps,sequences=seq,measurements=measurements,
                period_ns=self.period_ns,session=self.session,sent_monotonic=time.monotonic(),
                source='isaaclab_physx_imu',acceleration_convention='proper_acceleration_with_gravity')
        os.replace(temporary,self.destination)
