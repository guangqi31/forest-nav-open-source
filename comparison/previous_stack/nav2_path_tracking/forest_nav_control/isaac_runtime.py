"""G1 策略的可选 Nav2 入口；不改变未启用该模式时的原有行为。"""
import copy
import json
import math
import os
import time
from pathlib import Path
from .sim_endpoint import SimEndpoint
from .command_history import CommandHistory

ROOT=Path(__file__).resolve().parents[2]


class NavigationPathOverlay:
    """把实际 Nav2 全局路线绘制为 Isaac 视口中的绿色显示曲线。"""

    def __init__(self, ground_map):
        import numpy as np
        import omni.usd
        from pxr import Gf, Sdf, UsdGeom, UsdShade

        self.np = np
        stage = omni.usd.get_context().get_stage()
        self.curve = UsdGeom.BasisCurves.Define(
            stage, '/World/ForestNavigation/GlobalPlan')
        self.curve.CreateTypeAttr('linear')
        self.curve.CreateWrapAttr('nonperiodic')
        self.curve.CreateWidthsAttr([0.065])
        self.curve.SetWidthsInterpolation('constant')
        self.curve.CreateDisplayColorAttr([Gf.Vec3f(0.05, 1.0, 0.20)])
        material = UsdShade.Material.Define(
            stage, '/World/ForestNavigation/PlanMaterial')
        shader = UsdShade.Shader.Define(
            stage, '/World/ForestNavigation/PlanMaterial/Shader')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput(
            'diffuseColor', Sdf.ValueTypeNames.Color3f
        ).Set(Gf.Vec3f(0.02, 0.8, 0.08))
        shader.CreateInput(
            'emissiveColor', Sdf.ValueTypeNames.Color3f
        ).Set(Gf.Vec3f(0.02, 0.8, 0.08))
        material.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), 'surface')
        UsdShade.MaterialBindingAPI.Apply(
            self.curve.GetPrim()).Bind(material)
        self.markers = []
        for name, color in (('Start', (0.05, 0.5, 1.0)),
                            ('Goal', (1.0, 0.12, 0.05))):
            sphere = UsdGeom.Sphere.Define(
                stage, '/World/ForestNavigation/' + name)
            sphere.CreateRadiusAttr(0.11)
            sphere.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            sphere.AddTranslateOp()
            self.markers.append(sphere)
        self.ground_map = ground_map
        self.revision = -1
        self.display_points = 0
        self.last_display_points = 0
        self.last_path_points = 0
        self.route_seen = False
        self.capture_delay_frames = None
        self.capture_started = False
        self.clear()

    def _z(self, x, y):
        if self.ground_map is None:
            return 0.12
        elevation, origin, resolution = self.ground_map
        col, row = self.np.floor(
            (self.np.array([x, y]) - origin) / resolution).astype(int)
        if not (0 <= row < elevation.shape[0]
                and 0 <= col < elevation.shape[1]):
            raise ValueError('导航路线越出高程图')
        height = elevation[max(0, row - 1):row + 2,
                           max(0, col - 1):col + 2]
        if not self.np.isfinite(height).any():
            raise ValueError('导航路线落在未知地面')
        return float(self.np.nanmax(height)) + 0.12

    def clear(self):
        had_path = self.display_points > 0
        self.curve.GetPointsAttr().Set([])
        self.curve.GetCurveVertexCountsAttr().Set([])
        for marker in self.markers:
            marker.MakeInvisible()
        self.display_points = 0
        self.capture_delay_frames = None
        self.capture_started = False
        if had_path:
            print('[ForestNav] 导航路线已清除', flush=True)

    def show(self, points, revision):
        if revision == self.revision:
            return False
        if not points:
            self.clear()
            self.revision = revision
            self._write_verification(
                passed=True, message='路线已清除',
                current_display_points=0)
            return True
        from pxr import Gf
        draw = []
        for first, second in zip(points[:-1], points[1:]):
            count = max(2, math.ceil(math.dist(first, second) / 0.05) + 1)
            for x, y in self.np.linspace(first, second, count)[:-1]:
                draw.append(Gf.Vec3f(float(x), float(y), self._z(x, y)))
        x, y = points[-1]
        draw.append(Gf.Vec3f(float(x), float(y), self._z(x, y)))
        if len(draw) > 20000:
            raise ValueError('导航路线显示点过多')
        self.curve.GetPointsAttr().Set(draw)
        self.curve.GetCurveVertexCountsAttr().Set([len(draw)])
        for marker, point in zip(self.markers, (points[0], points[-1])):
            x, y = point
            marker.GetOrderedXformOps()[0].Set(
                Gf.Vec3d(float(x), float(y), self._z(x, y) + 0.08))
            marker.MakeVisible()
        self.display_points = len(draw)
        self.last_display_points = len(draw)
        self.last_path_points = len(points)
        self.route_seen = True
        self.revision = revision
        if self.capture_delay_frames is None:
            self.capture_delay_frames = 5
            self.capture_started = False
        self._write_verification(
            passed=True, message='实际 Nav2 路线已显示',
            path_points=len(points), display_points=len(draw))
        print(
            f'[ForestNav] 实际 Nav2 路线已显示：'
            f'{len(points)}路径点，{len(draw)}显示点',
            flush=True,
        )
        return True

    def _write_verification(self, **values):
        sensor_mesh = os.environ.get('FOREST_NAV_SENSOR_MESH')
        if not sensor_mesh:
            return
        output = Path(sensor_mesh).parent / 'path_overlay_verification.json'
        payload = {
            'passed': bool(values.pop('passed')),
            'revision': self.revision,
            'current_display_points': self.display_points,
            'display_points': self.last_display_points,
            'path_points': self.last_path_points,
            'route_seen': self.route_seen,
        }
        payload.update(values)
        capture = os.environ.get('FOREST_NAV_PATH_CAPTURE')
        if capture:
            payload['capture'] = capture
        output.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + '\n')

    def tick_capture(self, frames):
        if self.capture_delay_frames is None or self.capture_started:
            return
        if self.capture_delay_frames > 0:
            self.capture_delay_frames -= 1
            return
        self.capture_started = True
        capture = os.environ.get('FOREST_NAV_PATH_CAPTURE')
        if not capture:
            return
        try:
            from omni.kit.viewport.utility import (
                capture_viewport_to_file, get_active_viewport)
            Path(capture).parent.mkdir(parents=True, exist_ok=True)
            capture_viewport_to_file(get_active_viewport(), capture)
            self._write_verification(
                passed=True, message='实际 Nav2 路线已显示并请求视口截图',
                path_points=self.display_points)
        except Exception as exc:
            print(f'[ForestNav] 视口截图失败：{exc}', flush=True)
            self._write_verification(
                passed=False, message=f'视口截图失败：{exc}')


def configure_navigation(cfg):
    """外部导航使用连续物理仿真，取消训练式自动重置、随机指令和随机出生。"""
    from isaaclab.envs.mdp.commands import UniformVelocityCommandCfg
    cfg.scene.num_envs=1
    cfg.episode_length_s=1e10
    cfg.commands.base_velocity=UniformVelocityCommandCfg(
        asset_name='robot',resampling_time_range=(1e9,1e9),heading_command=False,
        rel_standing_envs=0.,debug_vis=False,
        ranges=UniformVelocityCommandCfg.Ranges(lin_vel_x=(0.,0.),lin_vel_y=(0.,0.),ang_vel_z=(0.,0.)))
    cfg.events.reset_base.params['pose_range']={key:(0.,0.) for key in ['x','y','yaw']}
    cfg.events.reset_base.params['velocity_range']={key:(0.,0.) for key in ['x','y','z','roll','pitch','yaw']}
    cfg.events.reset_robot_joints.params={'position_range':(0.,0.),'velocity_range':(0.,0.)}
    cfg.curriculum.terrain_levels=None
    # 训练式 episode 终止会在 env.step 内瞬移回出生点，而 LIO 仍在
    # 积分上一段运动。导航允许碰撞和摔倒，所有训练终止条件均关闭。
    for name in vars(cfg.terminations):
        if not name.startswith('_'):
            setattr(cfg.terminations, name, None)
    cfg.events.register_virtual_obstacles=None
    scene_usd=os.environ.get('FOREST_TERRAIN_SCENE')
    if scene_usd:
        # 三维场景联调入口：直接加载真实地形 USD，不从代价图构造障碍。
        # 出生区在新场景 map/world 原点，脚下地面高度由场景契约约定为0。
        path=Path(scene_usd).resolve()
        if not path.is_file():
            raise FileNotFoundError(f'三维林地 USD 不存在：{path}')
        manifest=json.loads((path.parent/'scene_manifest.json').read_text())
        if manifest['units']!='meter' or manifest['up_axis']!='Z' or manifest['T_world_map']!=[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]:
            raise ValueError('当前三维场景入口要求米制、Z向上、world与map重合')
        cfg.scene.terrain.terrain_type='usd'
        cfg.scene.terrain.usd_path=str(path)
        cfg.scene.terrain.terrain_generator=None
        cfg.scene.terrain.visual_material=None
        cfg.scene.sky_light.spawn.texture_file=None
        sensor_mesh=os.environ.get('FOREST_NAV_SENSOR_MESH')
        if sensor_mesh:
            # 只绑定实际支撑地面和碰撞障碍；纯视觉树冠不能污染策略深度输入。
            ground=cfg.scene.terrain.prim_path+'/terrain/Ground'
            cfg.scene.left_height_scanner.mesh_prim_paths=[ground]
            cfg.scene.right_height_scanner.mesh_prim_paths=[ground]
            cfg.scene.camera.mesh_prim_paths=[ground,'/World/ForestNavigationSensors/Obstacles']+[
                target for target in cfg.scene.camera.mesh_prim_paths
                if '/Robot/' in (target if isinstance(target,str) else target.prim_expr)]
        if os.environ.get('FOREST_LIDAR_ENABLED')=='1':
            from .lidar_sensor import configure_lidar
            configure_lidar(cfg)
        return
    terrain=copy.deepcopy(cfg.scene.terrain.terrain_generator)
    plane=copy.deepcopy(terrain.sub_terrains['perlin_rough'])
    plane.proportion=1.
    plane.noise_scale=[0.,0.]
    plane.wall_prob=[0.,0.,0.,0.]
    plane.flat_patch_sampling={}
    terrain.num_rows=terrain.num_cols=1
    terrain.size=(40.,40.)
    terrain.horizontal_scale=.2
    terrain.sub_terrains={'perlin_rough':plane}
    terrain.curriculum=False
    cfg.scene.terrain.terrain_generator=terrain
    cfg.scene.terrain.visual_material=None
    cfg.scene.sky_light.spawn.texture_file=None
    # 障碍由同一代价地图构建，策略的深度传感器也必须观察到它们。
    cfg.scene.camera.mesh_prim_paths.append('/World/Nav2Map/Obstacles')


def prepare_map_geometry():
    """将地图的100格转换为真实碰撞方块；高代价区保持可行平地，未知不造假墙。

    每行相邻障碍格合并成一个长方体，避免逐格创建上千个 USD 物体。
    地图起点(-8,-2)对应仿真世界(0,0)，与下方状态反馈使用同一平移。
    """
    if os.environ.get('FOREST_TERRAIN_SCENE'):
        # TerrainImporter加载实际USD。传感器专用网格来自同一USD碰撞几何，
        # 没有CollisionAPI、不可见，不会产生第二套物理障碍。
        filename=os.environ.get('FOREST_NAV_SENSOR_MESH')
        if filename:
            import numpy as np
            import omni.usd
            from pxr import UsdGeom
            with np.load(filename,allow_pickle=False) as data:
                points,triangles=data['points'],data['triangles']
            mesh=UsdGeom.Mesh.Define(omni.usd.get_context().get_stage(),'/World/ForestNavigationSensors/Obstacles')
            mesh.CreatePointsAttr(points);mesh.CreateFaceVertexCountsAttr([3]*len(triangles))
            mesh.CreateFaceVertexIndicesAttr(triangles.ravel());mesh.MakeInvisible()
            print(f'[ForestNav] 策略障碍网格已加载：{len(points)}点、{len(triangles)}三角形，无额外碰撞',flush=True)
        return
    import numpy as np
    import omni.usd
    from pxr import Gf, UsdGeom, UsdPhysics
    filename=os.environ.get('FOREST_NAV_MAP',str(ROOT/'forest_costmap_test/forest_costmap.npz'))
    with np.load(filename,allow_pickle=False) as data:
        grid,resolution,origin=data['cost'],float(data['resolution']),data['origin']
    stage=omni.usd.get_context().get_stage()
    vertices,indices=[],[]
    faces=[(0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),
           (1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)]
    for row,values in enumerate(grid):
        mask=np.r_[False,values==100,False].astype(int)
        starts=np.where(np.diff(mask)==1)[0]
        ends=np.where(np.diff(mask)==-1)[0]
        for start,end in zip(starts,ends):
            x0,x1=origin[0]+start*resolution+8,origin[0]+end*resolution+8
            y0,y1=origin[1]+row*resolution+2,origin[1]+(row+1)*resolution+2
            base=len(vertices)
            vertices.extend([(x0,y0,0),(x1,y0,0),(x1,y1,0),(x0,y1,0),
                             (x0,y0,2),(x1,y0,2),(x1,y1,2),(x0,y1,2)])
            indices.extend(base+i for face in faces for i in face)
    mesh=UsdGeom.Mesh.Define(stage,'/World/Nav2Map/Obstacles')
    mesh.CreatePointsAttr().Set(vertices)
    mesh.CreateFaceVertexCountsAttr().Set([3]*(len(indices)//3))
    mesh.CreateFaceVertexIndicesAttr().Set(indices)
    mesh.CreateDisplayColorAttr().Set([Gf.Vec3f(.18,.30,.10)])
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    print(f'[Nav2] 已从代价地图构建碰撞场景：{len(vertices)//8} 个合并障碍块',flush=True)


class IsaacNavigation:
    def __init__(self,env):
        self.env=env
        self.endpoint=SimEndpoint()
        self.command_term=env.unwrapped.command_manager.get_term('base_velocity')
        self.command_history=CommandHistory()
        self.robot=env.unwrapped.scene['robot']
        self.left_height_scanner=env.unwrapped.scene['left_height_scanner']
        self.right_height_scanner=env.unwrapped.scene['right_height_scanner']
        # 新三维场景 map 与 world 重合；旧联调场景仍沿用原有平移约定。
        self.map_translation=(0.,0.) if os.environ.get('FOREST_TERRAIN_SCENE') else (-8.,-2.)
        self.ground_map=None
        if os.environ.get('FOREST_NAV_SENSOR_MESH'):
            import numpy as np
            with np.load(os.environ['FOREST_NAV_MAP'],allow_pickle=False) as data:
                self.ground_map=(data['elevation'].copy(),data['origin'].copy(),float(data['resolution']))
        self.path_overlay=None
        self.path_overlay_notice=''
        try:
            self.path_overlay=NavigationPathOverlay(self.ground_map)
        except Exception as exc:
            self.path_overlay_notice=f'导航路线叠加初始化失败：{exc}'
            print('[ForestNav] '+self.path_overlay_notice,flush=True)
        self.lidar_exporter=None
        self.goal_preview=None
        if self.path_overlay is not None:
            from .goal_preview_overlay import GoalPreviewOverlay
            self.goal_preview=GoalPreviewOverlay(self.path_overlay._z)
        if os.environ.get('FOREST_LIDAR_ENABLED')=='1':
            from .lidar_sensor import LidarExporter
            self.lidar_exporter=LidarExporter(env)
        self.started=time.monotonic()
        self.frames=0
        self.next_tick=self.started
        self.applied=(0.,0.,0.)
        self.pre_step_motion=None
        self.publish_state(ready=False)
        if os.environ.get('FOREST_NAV_SENSOR_MESH'):
            from .forest_audit import ForestSafetyAudit
            self.forest_audit=ForestSafetyAudit(env,Path(os.environ['FOREST_NAV_SENSOR_MESH']).parent,self.ground_map)

    def publish_state(self,ready=True,fault=''):
        # IsaacLab 3 使用后端数组，.torch 获取现有张量，不改变物理状态。
        data=self.robot.data
        position=data.root_pos_w.torch[0].detach().cpu().tolist()
        # IsaacLab 3 的后端四元数为 xyzw；按旧版 wxyz 读取会把静止航向误报为180度。
        x,y,z,w=data.root_quat_w.torch[0].detach().cpu().tolist()
        yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
        velocity=data.root_lin_vel_w.torch[0].detach().cpu().tolist()
        angular=data.root_ang_vel_w.torch[0].detach().cpu().tolist()
        # 速度反馈投影到水平机体航向，避免俯仰把垂直速度混进 Nav2 平面速度。
        vx=math.cos(yaw)*velocity[0]+math.sin(yaw)*velocity[1]
        vy=-math.sin(yaw)*velocity[0]+math.cos(yaw)*velocity[1]
        # 不平地上必须用相对地面的躯干高度，不能把世界Z当作离地高度。
        ground_height=self.measured_ground_height()
        if ground_height is None:fault='terrain_height_unknown'
        elif position[2]-float(ground_height)<.45:
            fault='robot_fallen'
        if 1-2*(x*x+y*y)<.5:
            fault='robot_fallen'
        self.last_fault=fault
        self.endpoint.publish_state(1+self.frames*self.env.unwrapped.step_dt,
            [position[0]+self.map_translation[0],position[1]+self.map_translation[1],yaw],[vx,vy,angular[2]],ready=ready,fault=fault,
            locomotion=self.dual_policy.status if hasattr(self,'dual_policy') else None)
        return position

    def before_step(self,obs,command_obs_slice):
        self.pre_step_motion=(self.motion_snapshot()
                              if hasattr(self,'robot') else None)
        values=self.endpoint.read_command()
        self.applied=values
        self.command_term.vel_command_b[:]=self.command_term.vel_command_b.new_tensor(values)
        # 外部指令在推理前才到达，必须逐帧追加实际指令历史。
        # 不能用 repeat 把过去八帧全部改成最新值；否则停步时策略看不到减速过程。
        # 这里只替换速度这一项，其余环境观测及 command manager 的更新保持原样。
        width=int(command_obs_slice[1][0])
        history=self.command_history.append(values,width)
        obs[:,command_obs_slice[0]]=obs.new_tensor(history)
        if self.goal_preview is not None:
            self.goal_preview.tick()
        if self.path_overlay is not None:
            points,revision=self.endpoint.read_path()
            if revision>=0:
                try:
                    self.path_overlay.show(points,revision)
                except Exception as exc:
                    notice=f'导航路线显示失败：{exc}'
                    if notice!=self.path_overlay_notice:
                        self.path_overlay_notice=notice
                        print('[ForestNav] '+notice,flush=True)
                    self.path_overlay.clear()

    def publish_sensor_cloud(self):
        """只发送实际相机已命中的三维点，最多每0.5仿真秒一帧，不导出场景全图。"""
        if self.frames % max(1,round(.5/self.env.unwrapped.step_dt)):
            return
        import socket
        import numpy as np
        from .cloud_wire import encode_chunks, PORT
        if not hasattr(self,'cloud_socket'):
            self.cloud_socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        camera=self.env.unwrapped.scene['camera']
        _=camera.data  # 通过现有传感器属性触发必要的数据更新。
        points=camera.ray_hits_w.torch[0].detach().cpu().numpy()
        origin=camera.data.pos_w.torch[0].detach().cpu().numpy()
        robot_xy=self.robot.data.root_pos_w.torch[0,:2].detach().cpu().numpy()
        valid=np.isfinite(points).all(axis=1)
        ranges=np.linalg.norm(points-origin,axis=1)
        valid &= (ranges>.25)&(ranges<8.)
        valid &= np.linalg.norm(points[:,:2]-robot_xy,axis=1)>.45
        points=points[valid]
        if len(points):
            for packet in encode_chunks(points,1+self.frames*self.env.unwrapped.step_dt):
                self.cloud_socket.sendto(packet,('127.0.0.1',int(os.environ.get('FOREST_NAV_CLOUD_PORT',PORT))))

    def after_step(self,dones):
        self.frames+=1
        if self.lidar_exporter is not None:
            self.lidar_exporter.tick(1+self.frames*self.env.unwrapped.step_dt)
        if os.environ.get('FOREST_NAV_MAP_MODE')=='online' and os.environ.get('FOREST_NAV_CLOUD_SOURCE','sim_camera')=='sim_camera':
            self.publish_sensor_cloud()
        if hasattr(self,'forest_audit'):self.forest_audit.tick(self.frames)
        # 初始两秒保持零速度，让机器人站稳后才向导航公布 ready。
        position=self.publish_state(ready=self.frames*self.env.unwrapped.step_dt>=2)
        if self.frames%100==0:
            print(f'[Nav2] step={self.frames} world_xy={position[:2]} command={self.applied}',flush=True)
        if self.path_overlay is not None:
            self.path_overlay.tick_capture(self.frames)
        # 不让离线仿真无限快跑过导航消息；慢于实时则保持实际仿真时钟。
        self.next_tick+=self.env.unwrapped.step_dt
        time.sleep(max(0.,self.next_tick-time.monotonic()))

    def motion_snapshot(self):
        """记录根节点世界位姿、速度、倾斜和相对已知地面高度。"""
        import numpy as np
        position=self.robot.data.root_pos_w.torch[0].detach().cpu().tolist()
        velocity=self.robot.data.root_lin_vel_w.torch[0].detach().cpu().tolist()
        angular=self.robot.data.root_ang_vel_w.torch[0].detach().cpu().tolist()
        x,y,z,w=self.robot.data.root_quat_w.torch[0].detach().cpu().tolist()
        body_z_world=1-2*(x*x+y*y)
        tilt=math.acos(max(-1.,min(1.,body_z_world)))
        ground_height=self.measured_ground_height()
        return dict(
            sim_time=1+self.frames*self.env.unwrapped.step_dt,
            position_world_m=position,
            linear_velocity_world_mps=velocity,
            angular_velocity_world_radps=angular,
            root_quat_xyzw=[x,y,z,w],
            tilt_rad=tilt,
            ground_height_source='left/right_height_scanner',
            ground_height_m=ground_height,
            root_ground_clearance_m=(position[2]-ground_height
                                     if ground_height is not None else None),
        )

    def episode_reset_details(self):
        """记录终止项与步进前姿态，防止把通用 episode_reset 误判为摔倒。"""
        terms={}
        try:
            for name,values in self.env.unwrapped.termination_manager.get_active_iterable_terms(0):
                terms[name]=bool(values and float(values[0])>.5)
        except Exception as exc:
            terms['diagnostic_error']=str(exc)
        details=dict(
            sim_time=1+self.frames*self.env.unwrapped.step_dt,
            frame=self.frames,
            termination_terms=terms,
            pre_reset_motion=self.pre_step_motion,
            post_reset_motion=self.motion_snapshot(),
        )
        sensor_mesh=os.environ.get('FOREST_NAV_SENSOR_MESH')
        if sensor_mesh:
            output=Path(sensor_mesh).parent/'episode_reset_diagnostic.json'
            output.write_text(json.dumps(details,ensure_ascii=False,indent=2)+'\n')
        return details

    def measured_ground_height(self):
        """从左右脚高度扫描器读取实际支撑地面；无有效命中时返回 None。"""
        import numpy as np
        if not (hasattr(self,'left_height_scanner')
                and hasattr(self,'right_height_scanner')):
            return None
        heights=[]
        for scanner in (self.left_height_scanner,self.right_height_scanner):
            hits=scanner.data.ray_hits_w.torch[0].detach().cpu().numpy()
            hits=np.asarray(hits,dtype=np.float64).reshape(-1,3)
            valid=hits[np.isfinite(hits).all(axis=1),2]
            heights.extend(valid.tolist())
        return float(np.median(heights)) if heights else None

    def close(self):
        if hasattr(self,'forest_audit'):self.forest_audit.save()
        if hasattr(self,'cloud_socket'):self.cloud_socket.close()
        self.endpoint.close()
