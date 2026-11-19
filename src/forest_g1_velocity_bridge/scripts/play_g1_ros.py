"""把现有G1 ONNX推理入口连接到原生ROS速度订阅，不修改策略源码。"""
import importlib.util
import os
import sys
import time
from pathlib import Path

from forest_g1_velocity_bridge import G1VelocityAdapter

# 在创建SimulationApp前选用桥接①已有的DDS配置，不导入系统rclpy。
workspace = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(workspace / "src/forest_sensor_bridge/isaac"))
from sensor_graph import SensorBridge, configure_transport
configure_transport()

# 复用现有播放脚本的配置解析和ONNX加载函数，不调用旧导航运行分支。
source = Path(os.environ.get("G1_PLAY_SOURCE", "/home/ubuntu/robot_projects/forest_nav/InstinctLab/source/instinctlab/instinctlab/tasks/parkour/scripts/play.py"))
spec = importlib.util.spec_from_file_location("g1_existing_play", source)
play = importlib.util.module_from_spec(spec)
spec.loader.exec_module(play)


def run_ros_policy(env_cfg, agent_cfg, log_dir, resume_path):
    """只接线：外部速度观测→原ONNX策略→原环境step。"""
    import gymnasium as gym
    import torch
    from instinctlab.utils.wrappers import InstinctRlVecEnvWrapper
    from instinct_rl.utils.utils import get_subobs_by_components, get_subobs_size

    scene_path = os.environ.get("G1_FOREST_SCENE")
    if scene_path:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from forest_scene import configure_forest, attach_policy_mesh
        attach_policy_mesh(configure_forest(env_cfg, scene_path))
        print(f"[G1 SCENE] complex forest: {scene_path}", flush=True)
    # 原生物理仍按原配置步长推进，策略仍按原decimation推理。
    # RTX雷达为10Hz；将画面渲染从每个策略步改为每0.1秒一次，
    # 减少重复渲染，不更改雷达配置、IMU或策略Warp传感器的更新周期。
    env_cfg.sim.render_interval = round(0.1 / env_cfg.sim.dt)
    adapter = G1VelocityAdapter(topic=os.environ.get("G1_CMD_VEL_TOPIC", "/cmd_vel"))
    # 此入口只做ONNX推理。网络只使用policy组，critic/AMP观测用于训练，
    # 不应在每步继续计算，也不应让critic重复触发同一个速度订阅图。
    # policy组的观测项、顺序、8帧历史、噪声及尺度均沿用原配置。
    for group_name in ("critic", "amp_policy", "amp_reference"):
        setattr(env_cfg.observations, group_name, None)
    env_cfg.scene.motion_reference = None
    # 使用原生奖励管理器已有的零权重分支跳过26项训练奖励计算。
    # 保留原奖励组容器和返回形状，兼容现有包装器；奖励不进入ONNX网络。
    for group_cfg in vars(env_cfg.rewards).values():
        if group_cfg is not None and hasattr(group_cfg, "__dict__"):
            for term_cfg in vars(group_cfg).values():
                if hasattr(term_cfg, "weight"):
                    term_cfg.weight = 0.0
    adapter.configure_observations(env_cfg, group_names=("policy",))
    # 交互仿真不因上游训练任务的回合时限、摔倒条件自动重置。
    # 仅清空宿主配置中的终止项，不改开源终止管理器或添加判断算法。
    for name in list(vars(env_cfg.terminations)):
        if not name.startswith("_"):
            setattr(env_cfg.terminations, name, None)
    env = InstinctRlVecEnvWrapper(gym.make(play.args_cli.task, cfg=env_cfg))
    # 当前入口固定单台G1。传感器安装到真实torso_link刚体的子级，随身体运动。
    # 安装坐标和FAST-LIO外参共用同一sensors.yaml，不能挂在世界坐标的空Xform上。
    body_path = env.unwrapped.scene.env_prim_paths[0] + "/Robot/torso_link"
    sim = env.unwrapped.sim
    from isaacsim.core.simulation_manager import SimulationManager
    sensors = SensorBridge(
        body_path, config_path=workspace / "src/forest_sensor_bridge/config/sensors.yaml",
        # 使用原生物理时间，与原生IMU和RTX雷达的时间源一致，包含初始化物理步。
        time_provider=SimulationManager.get_simulation_time,
        physics_dt=sim.get_physics_dt())
    print(f"[G1 SENSORS] mounted on {body_path}; lidar=10Hz imu=200Hz", flush=True)
    adapter.connect()
    # 观测结构在环境创建后固定，只解析一次，推理循环不重复构建切片信息。
    obs_segments = env.get_obs_segments()
    policy = play.load_parkour_onnx_model(
        model_dir=os.path.join(log_dir, "exported"),
        get_subobs_func=lambda obs: get_subobs_by_components(
            obs, agent_cfg.policy.encoder_configs.depth_encoder.component_names,
            obs_segments, temporal=True),
        depth_shape=obs_segments["depth_image"],
        proprio_slice=slice(0, get_subobs_size(obs_segments, [
            "base_lin_vel", "base_ang_vel", "projected_gravity", "velocity_commands",
            "joint_pos", "joint_vel", "actions"])),
    )
    obs, _ = env.get_observations()
    print("[G1 ROS] READY: /cmd_vel TwistStamped -> velocity_commands -> ONNX -> env.step", flush=True)
    # 持续执行原策略；不按速度、任务状态、时间或摔倒结果退出。
    # 这里只适配仿真与宿主的时间节拍，不改速度指令或策略输出。
    # 计算快于实时才等待；计算慢时继续正常物理步，不跳步、不扩大dt、
    # 不伪造传感器时间，也不因落后现实时间而退出或重置。
    # 每一步独立计算等待时间，不累计落后量。暂停或计算变慢后，
    # 后续步骤不会为了追赶旧期限而连续加速；耗时超过预算时继续正常步进。
    # 这里只能防止单步计算快于实时，不能保证重负载下一定达到1:1。
    step_period = env.unwrapped.step_dt
    while True:
        step_started = time.monotonic()
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)
        remaining = step_period - (time.monotonic() - step_started)
        if remaining > 0:
            time.sleep(remaining)


# 原main继续负责任务配置及仿真启动，只替换本进程里的运行入口。
# 不写回原play.py，不启用旧nav2_control或键盘覆盖。
play._run_play = run_ros_policy
play.main()
