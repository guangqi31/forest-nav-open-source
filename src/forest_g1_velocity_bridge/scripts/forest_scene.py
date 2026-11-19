"""复用既有复杂林地USD，只有场景配置和策略传感器网格接线。"""
from pathlib import Path
import json


def configure_forest(env_cfg, scene_path):
    scene = Path(scene_path).resolve()
    manifest = json.loads((scene.parent / 'scene_manifest.json').read_text())
    # 原训练PoseVelocityCommand在构造时依赖生成式地形；USD场景改用原生速度命令容器。
    # 实际三列仍由ROS观测适配器写入，不使用该容器生成控制算法或限制外部速度。
    from isaaclab.envs.mdp import UniformVelocityCommandCfg
    resampling = env_cfg.commands.base_velocity.resampling_time_range
    env_cfg.commands.base_velocity = UniformVelocityCommandCfg(
        asset_name='robot', resampling_time_range=resampling, heading_command=False,
        rel_standing_envs=0., debug_vis=False,
        ranges=UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(0., 0.), lin_vel_y=(0., 0.), ang_vel_z=(0., 0.)))
    env_cfg.scene.terrain.terrain_type = 'usd'
    env_cfg.scene.terrain.usd_path = str(scene)
    env_cfg.scene.terrain.terrain_generator = None
    env_cfg.scene.terrain.visual_material = None
    env_cfg.scene.terrain.virtual_obstacles = {}
    env_cfg.curriculum.terrain_levels = None
    env_cfg.events.register_virtual_obstacles = None
    # 场景既有平坦出生区，脚下支撑高度为0，机器人根高度保留原策略配置。
    spawn = manifest['spawn']['support_xyz_m']
    pos = list(env_cfg.scene.robot.init_state.pos)
    pos[0], pos[1], pos[2] = spawn[0], spawn[1], spawn[2] + pos[2]
    env_cfg.scene.robot.init_state.pos = tuple(pos)
    env_cfg.events.reset_base.params['pose_range'] = {k: (0., 0.) for k in ('x', 'y', 'yaw')}
    env_cfg.events.reset_base.params['velocity_range'] = {k: (0., 0.) for k in ('x','y','z','roll','pitch','yaw')}
    env_cfg.scene.sky_light.spawn.texture_file = None
    ground = env_cfg.scene.terrain.prim_path + '/terrain/Ground'
    env_cfg.scene.left_height_scanner.mesh_prim_paths = [ground]
    env_cfg.scene.right_height_scanner.mesh_prim_paths = [ground]
    env_cfg.scene.camera.mesh_prim_paths = [ground, '/World/ForestPolicyGeometry/Obstacles'] + [
        target for target in env_cfg.scene.camera.mesh_prim_paths
        if '/Robot/' in (target if isinstance(target, str) else target.prim_expr)]
    return scene.parent / 'collision_geometry.npz'


def attach_policy_mesh(mesh_path):
    # NPZ来自同一场景的既有碰撞几何，仅供原策略Warp射线查询。
    # 不加CollisionAPI，不生成额外物理障碍，也不向上层提供真值地图。
    import numpy as np
    import omni.usd
    from pxr import UsdGeom
    with np.load(mesh_path, allow_pickle=False) as data:
        points, triangles = data['obstacle_points'], data['obstacle_triangles']
    mesh = UsdGeom.Mesh.Define(omni.usd.get_context().get_stage(), '/World/ForestPolicyGeometry/Obstacles')
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([3] * len(triangles))
    mesh.CreateFaceVertexIndicesAttr(triangles.ravel())
    mesh.MakeInvisible()
