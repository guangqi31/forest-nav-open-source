"""Script to play a checkpoint of an RL agent from Instinct-RL."""

import gymnasium as gym
import os
import re
import subprocess
import torch
from typing import Callable

from instinct_rl.runners import OnPolicyRunner
from instinct_rl.utils.utils import get_obs_slice, get_subobs_by_components, get_subobs_size

from isaaclab.app import launch_simulation

import instinctlab.tasks  # noqa: F401
from instinctlab.utils import instinct_rl_cli as cli

parser = cli.build_play_parser()
# custom play arguments
parser.add_argument("--useonnx", action="store_true", default=False, help="Use the exported ONNX model for inference.")
parser.add_argument("--keyboard_control", action="store_true", default=False, help="Enable keyboard control.")
parser.add_argument("--keyboard_linvel_step", type=float, default=0.5, help="Linear velocity change per keyboard step.")
parser.add_argument("--keyboard_angvel", type=float, default=1.0, help="Angular velocity set by keyboard.")
# Nav2 为可选入口，原有键盘和独立播放方式保持原样。
parser.add_argument("--nav2_control", action="store_true", help="通过本机状态桥接收 Nav2 速度。")
parser.add_argument("--nav2_max_steps", type=int, default=0, help="导航联调的最大步数，0表示持续运行。")
args_cli, overrides = cli.parse_cli(parser, require_run_name=False)
if args_cli.nav2_control and args_cli.keyboard_control:
    parser.error("Nav2 与键盘不能同时控制速度")

# keyboard control requires the kit visualizer for keyboard input
if args_cli.keyboard_control and not any(
    token.startswith("visualizer=") and "kit" in token.split("=", 1)[1].split(",") for token in overrides
):
    parser.error("--keyboard_control requires --viz kit so keyboard input is available.")

cli.wait_for_debugger(args_cli.debug)


def load_parkour_onnx_model(
    model_dir: str, get_subobs_func: Callable, depth_shape: tuple, proprio_slice: slice
) -> Callable:
    """Load the Parkour ONNX policy."""
    import numpy as np

    import onnxruntime as ort

    ort_providers = ort.get_available_providers()
    # 单机器人小批量推理不需要每个模型各自创建大量工作线程。
    # 默认线程池在两次控制调用之间忙等，会抢占物理仿真与ROS调度；
    # 这里只调整执行线程，不改变模型权重、观测、动作或策略调用顺序。
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    encoder = ort.InferenceSession(os.path.join(model_dir, "0-depth_encoder.onnx"), options, providers=ort_providers)
    actor = ort.InferenceSession(os.path.join(model_dir, "actor.onnx"), options, providers=ort_providers)
    actor_input_name = actor.get_inputs()[0].name

    def policy(obs: torch.Tensor) -> torch.Tensor:
        depth_image_input = get_subobs_func(obs)
        depth_image_input = depth_image_input.cpu().numpy()
        depth_image_input = depth_image_input.reshape((-1, *depth_shape))
        depth_image_output = encoder.run(None, {encoder.get_inputs()[0].name: depth_image_input})[0]
        actor_input = np.concatenate(
            [
                obs.cpu().numpy()[:, proprio_slice],
                depth_image_output,
            ],
            axis=1,
        )
        actor_output = actor.run(None, {actor_input_name: actor_input})[0]
        return torch.from_numpy(actor_output).to(obs.device)

    return policy


def main():
    """Play with Instinct-RL agent."""
    # Resolve the run folder from the task defaults, Hydra and the CLI before any saved YAML is layered.
    env_cfg, agent_cfg = cli.resolve_play_cfg(args_cli.task, overrides)
    cli.update_instinct_rl_cfg(agent_cfg, args_cli)

    # specify directory for logging experiments
    log_root_path = os.path.abspath(os.path.join(cli.LOG_ROOT, agent_cfg.experiment_name))
    agent_cfg.load_run = args_cli.load_run
    if agent_cfg.load_run is not None:
        if args_cli.useonnx:
            # The ONNX models define the run; a PyTorch checkpoint is optional and only used for comparison.
            print(f"[INFO] Loading experiment from directory: {log_root_path}")
            onnx_actor_path = cli.checkpoint_path(
                log_root_path, agent_cfg.load_run, "actor.onnx", other_dirs=["exported"]
            )
            log_dir = os.path.dirname(os.path.dirname(onnx_actor_path))
            if any(re.match(agent_cfg.load_checkpoint, f) for f in os.listdir(log_dir)):
                resume_path = cli.checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)
            else:
                print(f"[INFO] No PyTorch checkpoint in {log_dir}; running the ONNX policy without comparison.")
                resume_path = None
        else:
            log_dir, resume_path = cli.play_run_dir(args_cli, agent_cfg)
    elif not args_cli.no_resume:
        raise RuntimeError(
            f"\033[91m[ERROR] No checkpoint specified and play.py resumes from a checkpoint by default. Please specify"
            f" a checkpoint to resume from using --load_run or use --no_resume to disable this behavior.\033[0m"
        )
    else:
        print(f"[INFO] No experiment directory specified. Using default: {log_root_path}")
        log_dir = os.path.join(log_root_path, agent_cfg.run_name + "_play")
        resume_path = "model_scratch.pt"
    if args_cli.exportonnx and resume_path is None:
        raise RuntimeError(
            f"--exportonnx needs a PyTorch checkpoint matching '{agent_cfg.load_checkpoint}' in {log_dir}."
        )
    video_prefix = cli.checkpoint_video_prefix(resume_path)

    env_yaml, agent_yaml = cli.load_saved_cfgs(args_cli, log_dir)
    if env_yaml is not None or agent_yaml is not None:
        env_cfg, agent_cfg = cli.resolve_play_cfg(args_cli.task, overrides, env_yaml=env_yaml, agent_yaml=agent_yaml)
    cli.apply_play_cli(args_cli, env_cfg, agent_cfg)
    agent_cfg.load_run = args_cli.load_run
    env_cfg.log_dir = log_dir

    if args_cli.keyboard_control:
        env_cfg.scene.num_envs = 1
        env_cfg.episode_length_s = 1e10

    if args_cli.nav2_control:
        from forest_nav_control.isaac_runtime import configure_navigation

        configure_navigation(env_cfg)

    cli.pre_launch_video_config(
        env_cfg,
        args_cli,
        step_offset=args_cli.video_start_step,
        prefix=video_prefix,
    )
    with launch_simulation(env_cfg, args_cli):
        cli.apply_video_recording(env_cfg, args_cli, os.path.join(log_dir, "videos", "play"))
        return _run_play(env_cfg, agent_cfg, log_dir, resume_path)


def _run_play(env_cfg, agent_cfg, log_dir: str, resume_path: str | None):
    """Run Parkour policy inference inside an active simulation runtime."""
    from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent

    from instinctlab.utils.wrappers import InstinctRlVecEnvWrapper

    if args_cli.keyboard_control:
        import carb.input
        import omni.appwindow
        from carb.input import KeyboardEventType

    if args_cli.nav2_control:
        from forest_nav_control.isaac_runtime import prepare_map_geometry

        prepare_map_geometry()

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg)

    # convert to single-agent instance if required by the RL algorithm
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    # wrap around environment for instinct-rl
    env = InstinctRlVecEnvWrapper(env)

    # load previously trained model
    ppo_runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    if agent_cfg.load_run is not None and resume_path is not None:
        print(f"[INFO]: Loading model checkpoint from: {resume_path}")
        ppo_runner.load(resume_path)

    # obtain the trained policy for inference
    if args_cli.sample:
        policy = ppo_runner.alg.actor_critic.act
    else:
        policy = ppo_runner.get_inference_policy(device=env.unwrapped.device)

    # export policy to onnx/jit
    if agent_cfg.load_run is not None:
        export_model_dir = os.path.join(log_dir, "exported")
        if args_cli.exportonnx:
            assert env.unwrapped.num_envs == 1, "Exporting to ONNX is only supported for single environment."
            if not os.path.exists(export_model_dir):
                os.makedirs(export_model_dir)
            obs, _ = env.get_observations()
            ppo_runner.alg.actor_critic.export_as_onnx(obs, export_model_dir)

    # use the exported model for inference
    if args_cli.useonnx:
        # NOTE: This is only applicable with parkour task
        onnx_policy = load_parkour_onnx_model(
            model_dir=os.path.join(log_dir, "exported"),
            get_subobs_func=lambda obs: get_subobs_by_components(
                obs,
                agent_cfg.policy.encoder_configs.depth_encoder.component_names,
                env.get_obs_segments(),
                temporal=True,
            ),
            depth_shape=env.get_obs_segments()["depth_image"],
            proprio_slice=slice(
                0,
                get_subobs_size(
                    env.get_obs_segments(),
                    [
                        "base_lin_vel",
                        "base_ang_vel",
                        "projected_gravity",
                        "velocity_commands",
                        "joint_pos",
                        "joint_vel",
                        "actions",
                    ],
                ),
            ),
        )

    override_command = torch.zeros(env.num_envs, 3, device=env.device)
    command_obs_slice = get_obs_slice(env.get_obs_segments(), "velocity_commands")

    def on_keyboard_input(e):
        if e.type not in (KeyboardEventType.KEY_PRESS, KeyboardEventType.KEY_REPEAT):
            return
        if e.input == carb.input.KeyboardInput.W:
            override_command[:, 0] += args_cli.keyboard_linvel_step
        if e.input == carb.input.KeyboardInput.S:
            override_command[:, 0] -= args_cli.keyboard_linvel_step
        if e.input == carb.input.KeyboardInput.A:
            override_command[:, 1] += args_cli.keyboard_linvel_step
        if e.input == carb.input.KeyboardInput.D:
            override_command[:, 1] -= args_cli.keyboard_linvel_step
        if e.input == carb.input.KeyboardInput.F:
            override_command[:, 2] = args_cli.keyboard_angvel
        if e.input == carb.input.KeyboardInput.G:
            override_command[:, 2] = -args_cli.keyboard_angvel
        if e.input == carb.input.KeyboardInput.R:
            override_command[:, 2] = 0.0
        if e.input == carb.input.KeyboardInput.X:
            override_command[:] = 0.0

    if args_cli.keyboard_control:
        app_window = omni.appwindow.get_default_app_window()
        keyboard = app_window.get_keyboard()
        input_interface = carb.input.acquire_input_interface()
        input_interface.subscribe_to_keyboard_events(keyboard, on_keyboard_input)

    # reset environment
    obs, _ = env.get_observations()
    nav2_bridge = None
    if args_cli.nav2_control:
        from forest_nav_control.isaac_runtime import IsaacNavigation

        nav2_bridge = IsaacNavigation(env)
    dual_policy = None
    if nav2_bridge is not None and os.environ.get("FOREST_NAV_DUAL_POLICY", "0") == "1":
        if not args_cli.useonnx or resume_path is not None:
            raise ValueError("双策略导航要求直接加载ONNX权重，不与PyTorch比较模式混用")
        from forest_nav_control.dual_policy import DualPolicy

        stand_dir = os.environ.get("FOREST_NAV_STAND_MODEL", os.path.join(os.path.dirname(log_dir), "stand_onboard"))
        dual_policy = DualPolicy(onnx_policy, log_dir, stand_dir, env.get_obs_segments())
        nav2_bridge.dual_policy = dual_policy
    timestep = 0
    # 导航异常或用户取消时，同样释放桥接端口并关闭仿真。
    try:
        while True:
            # run everything in inference mode
            with torch.inference_mode():
                # agent stepping
                if nav2_bridge is not None:
                    nav2_bridge.before_step(obs, command_obs_slice)
                if args_cli.keyboard_control:
                    obs[:, command_obs_slice[0]] = override_command.repeat(1, command_obs_slice[1][0] // 3)
                if dual_policy is not None:
                    motion = nav2_bridge.motion_snapshot()
                    velocity = motion["linear_velocity_world_mps"]
                    actions = dual_policy(
                        obs, nav2_bridge.applied, (velocity[0] ** 2 + velocity[1] ** 2) ** 0.5,
                        motion["angular_velocity_world_radps"][2], motion["tilt_rad"], env.unwrapped.step_dt,
                    )
                elif args_cli.useonnx and resume_path is None:
                    actions = onnx_policy(obs)
                elif args_cli.useonnx:
                    torch_actions = policy(obs)
                    actions = onnx_policy(obs)
                    if (actions - torch_actions).abs().max() > 1e-5:
                        print(
                            "[INFO]: ONNX model and PyTorch model have a difference of"
                            f" {(actions - torch_actions).abs().max()} in actions at joint"
                            f" {((actions - torch_actions).abs() > 1e-5).nonzero(as_tuple=True)[0]}"
                        )
                else:
                    actions = policy(obs)
                if timestep < args_cli.zero_act_until:
                    actions[:] = 0.0
                # env stepping
                obs, rewards, dones, infos = env.step(actions)
            timestep += 1
            if nav2_bridge is not None:
                nav2_bridge.after_step(dones)
                if args_cli.nav2_max_steps and timestep >= args_cli.nav2_max_steps:
                    break
    
            # exit the loop if video_length is meet
            if args_cli.video:
                # Exit the play loop after recording one video
                if timestep == args_cli.video_length:
                    break
    finally:
        if nav2_bridge is not None:
            nav2_bridge.close()
        env.close()

    if args_cli.video:
        recorder = env_cfg.video_recorders[0]
        subprocess.run(["code", "-r", os.path.join(recorder.output_dir, f"{recorder.output_filename_prefix}_0000.mp4")])


if __name__ == "__main__":
    main()
