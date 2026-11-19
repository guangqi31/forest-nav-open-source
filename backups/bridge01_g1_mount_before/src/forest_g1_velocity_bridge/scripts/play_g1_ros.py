"""把现有G1 ONNX推理入口连接到原生ROS速度订阅，不修改策略源码。"""
import importlib.util
import os
from pathlib import Path

from forest_g1_velocity_bridge import G1VelocityAdapter

# 在创建SimulationApp前选用桥接①已有的DDS配置，不导入系统rclpy。
workspace = Path(__file__).resolve().parents[3]
os.environ.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE", str(
    workspace / "src/forest_sensor_bridge/config/fastdds_large_points.xml"))

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

    adapter = G1VelocityAdapter(topic=os.environ.get("G1_CMD_VEL_TOPIC", "/cmd_vel"))
    adapter.configure_observations(env_cfg)
    # 交互仿真不因上游训练任务的回合时限、摔倒条件自动重置。
    # 仅清空宿主配置中的终止项，不改开源终止管理器或添加判断算法。
    for name in list(vars(env_cfg.terminations)):
        if not name.startswith("_"):
            setattr(env_cfg.terminations, name, None)
    env = InstinctRlVecEnvWrapper(gym.make(play.args_cli.task, cfg=env_cfg))
    adapter.connect()
    policy = play.load_parkour_onnx_model(
        model_dir=os.path.join(log_dir, "exported"),
        get_subobs_func=lambda obs: get_subobs_by_components(
            obs, agent_cfg.policy.encoder_configs.depth_encoder.component_names,
            env.get_obs_segments(), temporal=True),
        depth_shape=env.get_obs_segments()["depth_image"],
        proprio_slice=slice(0, get_subobs_size(env.get_obs_segments(), [
            "base_lin_vel", "base_ang_vel", "projected_gravity", "velocity_commands",
            "joint_pos", "joint_vel", "actions"])),
    )
    obs, _ = env.get_observations()
    print("[G1 ROS] READY: /cmd_vel TwistStamped -> velocity_commands -> ONNX -> env.step", flush=True)
    # 持续执行原策略；不按速度、任务状态、时间或摔倒结果退出。
    while True:
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)


# 原main继续负责任务配置及仿真启动，只替换本进程里的运行入口。
# 不写回原play.py，不启用旧nav2_control或键盘覆盖。
play._run_play = run_ros_policy
play.main()
