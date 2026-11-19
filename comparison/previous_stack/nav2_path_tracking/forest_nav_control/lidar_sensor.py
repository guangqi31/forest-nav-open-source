"""G1独立3D激光雷达：调用IsaacLab射线传感器，不读取深度图或地图格子。

此处是理想几何扫描：每帧所有激光束共用一个采集时刻，包含最近命中、遮挡和
量程裁剪；暂不模拟逐束扫描运动畸变、反射强度、透射、噪声和多回波。
Kit与ROS的Python版本不同，采用本机原子NPZ文件交接最新一帧，避免混装环境。
"""
import json
import os
import time
import uuid
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]/'forest_lidar_perception'


def parameters():
    return json.loads((ROOT/'config/lidar.json').read_text())


def configure_lidar(cfg):
    from isaaclab.sensors import MultiMeshRayCasterCfg
    from isaaclab.sensors.ray_caster.patterns import LidarPatternCfg
    settings=parameters()
    # 地面、树冠、树干、石块与倒木都从实际USD几何射线求交，不能只扫碰撞代理。
    # 自遮挡对象复用现有机器人网格路径，但不读取相机深度值。
    robot_targets=[target for target in cfg.scene.camera.mesh_prim_paths
                   if '/Robot/' in (target if isinstance(target,str) else target.prim_expr)]
    from .imu_sensor import configure_imu
    configure_imu(cfg)
    cfg.scene.navigation_lidar=MultiMeshRayCasterCfg(
        prim_path=cfg.scene.camera.prim_path,
        update_period=1./settings['frequency_hz'],
        mesh_prim_paths=[cfg.scene.terrain.prim_path+'/terrain']+robot_targets,
        ray_alignment='base',
        offset=MultiMeshRayCasterCfg.OffsetCfg(pos=tuple(settings['offset_torso_m'])),
        pattern_cfg=LidarPatternCfg(channels=settings['channels'],
            vertical_fov_range=tuple(settings['vertical_fov_deg']),
            horizontal_fov_range=(0.,360.),horizontal_res=settings['horizontal_resolution_deg']),
        max_distance=settings['range_max_m'],update_mesh_ids=True,debug_vis=False)


def rotation_xyzw(quaternion):
    x,y,z,w=map(float,quaternion)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


class LidarExporter:
    def __init__(self,env):
        self.env=env.unwrapped
        self.sensor=self.env.scene['navigation_lidar']
        self.settings=parameters()
        self.destination=Path(os.environ['FOREST_LIDAR_FRAME_FILE'])
        self.destination.parent.mkdir(parents=True,exist_ok=True)
        self.session=uuid.uuid4().hex
        self.sequence=0
        self.last_time=-1.
        self.report_path=self.destination.parent/'sensor_status.json'
        self.progress_path=self.destination.with_name('lidar_progress.json')
        from .imu_sensor import ImuExporter
        self.imu=ImuExporter(self.env,self.session,self.destination.parent)

    def tick(self,stamp):
        # 每一步先报告轻量进度，独立于5Hz仿真采样。若后续GPU计算卡住，心跳也
        # 会停止，桥接端仍按0.5墙钟秒失联；不能用旧雷达帧伪造仿真还在推进。
        progress_tmp=self.progress_path.with_suffix('.tmp')
        progress_tmp.write_text(json.dumps(dict(stamp=float(stamp),session=self.session,
            sent_monotonic=time.monotonic())))
        os.replace(progress_tmp,self.progress_path)
        self.imu.flush()
        if stamp-self.last_time < 1./self.settings['frequency_hz']-1e-6:return
        self.last_time=stamp
        data=self.sensor.data
        q=data.quat_w.torch[0].detach().cpu().numpy()
        matrix=rotation_xyzw(q)
        # RayCaster的cfg.offset已烘焙进射线起点，data.pos_w仍是所挂载机身帧原点。
        origin=data.pos_w.torch[0].detach().cpu().numpy()+matrix@np.asarray(self.settings['offset_torso_m'])
        world=data.ray_hits_w.torch[0].detach().cpu().numpy()
        # 第0个目标是完整林地；后续目标是机器人。丢弃自身回波但保留其遮挡。
        mesh_ids=data.ray_mesh_ids.torch[0].detach().cpu().numpy()
        valid=np.isfinite(world).all(axis=1)&(mesh_ids==0)
        finite_count=int(valid.sum())
        indices=np.flatnonzero(valid)
        world=world[valid]
        distances=np.linalg.norm(world-origin,axis=1)
        in_range=(distances>=self.settings['range_min_m'])&(distances<=self.settings['range_max_m'])
        indices=indices[in_range];world=world[in_range]
        horizontal_count=int(data.ray_hits_w.torch.shape[1])//self.settings['channels']
        rings=(indices//horizontal_count).astype(np.uint16)
        local=((world-origin)@matrix).astype(np.float32)
        robot=self.env.scene['robot'].data
        body_pose=np.r_[robot.root_pos_w.torch[0].detach().cpu().numpy(),robot.root_quat_w.torch[0].detach().cpu().numpy()]
        self.sequence+=1
        temporary=self.destination.with_suffix('.tmp')
        with temporary.open('wb') as stream:
            np.savez_compressed(stream,points=local,world_points=world.astype(np.float32),
                sensor_pose=np.r_[origin,q],body_pose=body_pose,
                rings=rings,point_time=np.zeros(len(local),dtype=np.float32),
                stamp_ns=round(float(stamp)*1e9),stamp=float(stamp),sent_monotonic=time.monotonic(),sequence=self.sequence,session=self.session)
        os.replace(temporary,self.destination)
        if self.sequence==1 or self.sequence%25==0:
            self.report_path.write_text(json.dumps(dict(source='isaaclab_multimesh_raycaster',
                frame='lidar_link',sequence=self.sequence,points=len(local),
                rays=int(data.ray_hits_w.torch.shape[1]),raw_finite_hits=finite_count,
                raw_range_min=None if len(distances)==0 else float(distances.min()),
                raw_range_max=None if len(distances)==0 else float(distances.max()),settings=self.settings,
                sensor_pose_xyzw=np.r_[origin,q].tolist(),uses_camera_depth=False),ensure_ascii=False,indent=2))
            print(f'[LiDAR] frame={self.sequence} hits={len(local)} stamp={stamp:.3f}',flush=True)
