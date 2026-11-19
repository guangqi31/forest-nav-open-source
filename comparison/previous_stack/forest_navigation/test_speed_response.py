"""直接标定现有 G1 策略对低速和混合指令的实际响应。

该测试不启动 Nav2，只通过策略现有的本机 UDP 协议发送固定速度并记录真实
根节点反馈。目的是区分“导航参数导致停住”和“策略对低速指令无响应”。
"""
import argparse
import json
import math
import os
import signal
import socket
import subprocess
import time
import uuid
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent


def simulation_environment():
    """去掉 ROS Python 环境，避免 Isaac Kit 与 ROS Humble 混装依赖。"""
    env = dict(os.environ)
    for name in ("ROS_DISTRO", "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH",
                 "CMAKE_PREFIX_PATH", "RMW_IMPLEMENTATION"):
        env.pop(name, None)
    if "LD_LIBRARY_PATH" in env:
        env["LD_LIBRARY_PATH"] = ":".join(
            part for part in env["LD_LIBRARY_PATH"].split(":")
            if "/opt/ros/" not in part
        )
    env.pop("FOREST_TERRAIN_SCENE", None)
    env.pop("FOREST_NAV_SENSOR_MESH", None)
    return env


def finite_vector(value, count):
    return (isinstance(value, list) and len(value) == count
            and all(isinstance(item, (int, float)) and math.isfinite(item)
                    for item in value))


class Probe:
    def __init__(self, state_port, command_port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("127.0.0.1", state_port))
        self.socket.setblocking(False)
        self.destination = ("127.0.0.1", command_port)
        self.session = None
        self.sequence = 0
        self.state = None
        self.state_received = None
        self.samples = []

    def poll(self):
        changed = False
        for _ in range(100):
            try:
                raw, _ = self.socket.recvfrom(8192)
            except BlockingIOError:
                break
            try:
                packet = json.loads(raw)
                if (not isinstance(packet, dict) or packet.get("version") != 1
                        or not isinstance(packet.get("session"), str)
                        or not isinstance(packet.get("sequence"), int)
                        or not finite_vector(packet.get("pose"), 3)
                        or not finite_vector(packet.get("velocity"), 3)
                        or not isinstance(packet.get("ready"), bool)
                        or not isinstance(packet.get("sim_time"), (int, float))
                        or not math.isfinite(packet["sim_time"])
                        or packet["sim_time"] <= 0):
                    continue
            except (ValueError, TypeError, KeyError, OverflowError):
                continue
            if self.state is not None:
                if packet["session"] == self.state["session"]:
                    if packet["sequence"] <= self.state["sequence"]:
                        continue
                elif time.monotonic() - self.state_received < 0.5:
                    continue
            self.state = packet
            self.state_received = time.monotonic()
            self.session = packet["session"]
            self.samples.append({
                "sim_time": packet["sim_time"],
                "pose": packet["pose"],
                "velocity": packet["velocity"],
                "ready": packet["ready"],
                "fault": packet.get("fault", ""),
            })
            changed = True
        return changed

    def send(self, command):
        if self.session is None:
            return
        self.sequence += 1
        packet = {
            "version": 1,
            "session": self.session,
            "sequence": self.sequence,
            "sent_ns": time.monotonic_ns(),
            "command": list(command),
        }
        self.socket.sendto(
            json.dumps(packet, allow_nan=False, separators=(",", ":")).encode(),
            self.destination,
        )

    def close(self):
        self.socket.close()


def make_open_plane_map(path):
    """生成只用于平地标定的无碰撞地图。"""
    size = 320
    np.savez(
        path,
        resolution=np.array(0.1),
        origin=np.array([-16.0, -16.0]),
        frame_id=np.array("map"),
        cost=np.zeros((size, size), dtype=np.int16),
    )


def wait_for_ready(probe, policy, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if policy.poll() is not None:
            raise RuntimeError(
                f"G1 策略启动失败，退出码 {policy.returncode}；请检查策略日志"
            )
        probe.poll()
        if (probe.state is not None and probe.state["ready"]
                and time.monotonic() - probe.state_received < 0.5):
            return
        time.sleep(0.02)
    raise TimeoutError("G1 策略未在时限内报告 ready")


def command_at(profile, elapsed, phase_index):
    if callable(profile):
        return profile(elapsed, phase_index)
    return profile


def hold_command(probe, policy, command_profile, sim_duration, phase_index,
                 timeout=90):
    """持续发送命令，返回该阶段开始和结束时的仿真状态。"""
    start_index = len(probe.samples)
    start_time = time.monotonic()
    start_sim = None
    end_sim = None
    deadline = start_time + timeout
    while time.monotonic() < deadline:
        if policy.poll() is not None:
            raise RuntimeError(
                f"速度阶段中 G1 策略退出，退出码 {policy.returncode}"
            )
        changed = probe.poll()
        now = time.monotonic()
        state = probe.state
        if state is None or now - probe.state_received > 0.5 or not state["ready"]:
            raise RuntimeError("策略状态失效")
        if state.get("fault"):
            raise RuntimeError(f"策略报告故障：{state['fault']}")
        if start_sim is None:
            start_sim = state["sim_time"]
        end_sim = state["sim_time"]
        if end_sim - start_sim >= sim_duration:
            break
        elapsed = state["sim_time"] - start_sim
        probe.send(command_at(command_profile, elapsed, phase_index))
        # 状态包通常等于物理步频；不额外等待可避免指令超时。
        if not changed:
            time.sleep(0.005)
    else:
        raise TimeoutError(f"速度阶段未获得 {sim_duration:g} 秒仿真时间")
    probe.send((0.0, 0.0, 0.0))
    return probe.samples[start_index:], start_sim, end_sim


def summarize_phase(name, command, samples, start_sim, end_sim):
    settled = [
        sample for sample in samples
        if sample["sim_time"] >= start_sim + 0.6 * (end_sim - start_sim)
    ]
    if not settled:
        settled = samples[-1:]
    first = samples[0]
    last = settled[-1]
    dt = max(last["sim_time"] - first["sim_time"], 1e-6)
    displacement = math.dist(first["pose"][:2], last["pose"][:2])
    mean_vx = float(np.mean([sample["velocity"][0] for sample in settled]))
    mean_abs_wz = float(np.mean([abs(sample["velocity"][2]) for sample in settled]))
    yaw_delta = math.atan2(
        math.sin(last["pose"][2] - first["pose"][2]),
        math.cos(last["pose"][2] - first["pose"][2]),
    )
    requested_motion = math.hypot(command[0], command[2])
    achieved_motion = (abs(mean_vx) if abs(command[0]) > 0
                       else abs(yaw_delta / dt))
    ratio = achieved_motion / requested_motion if requested_motion else 0.0
    return {
        "name": name,
        "command": list(command),
        "duration_sim_s": end_sim - start_sim,
        "sample_count": len(samples),
        "mean_actual_vx_mps": mean_vx,
        "mean_actual_abs_wz_radps": mean_abs_wz,
        "displacement_m": displacement,
        "yaw_change_rad": yaw_delta,
        "response_ratio": ratio,
        "start_pose": first["pose"],
        "end_pose": last["pose"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state-port", type=int, default=15491)
    parser.add_argument("--command-port", type=int, default=15492)
    parser.add_argument("--phase-seconds", type=float, default=3.0)
    parser.add_argument("--startup-timeout", type=float, default=240.0)
    parser.add_argument("--scene", choices=("forest", "plane"), default="forest")
    parser.add_argument("--profile",
                        choices=("basic", "mixed", "threshold", "signs",
                                 "angular-threshold", "all"),
                        default="basic")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--trajectory", type=Path)
    args = parser.parse_args()
    if not (0 < args.state_port < 65536 and 0 < args.command_port < 65536):
        parser.error("端口必须位于 1..65535")
    if args.state_port == args.command_port:
        parser.error("状态端口和命令端口不能相同")
    if not math.isfinite(args.phase_seconds) or args.phase_seconds < 1.0:
        parser.error("--phase-seconds 至少为 1 秒")

    output_path = args.output or (
        ROOT / "results" / f"speed_response_{args.profile}_verification.json"
    )
    trajectory_path = args.trajectory or (
        ROOT / "results" / f"speed_response_{args.profile}_trajectory.json"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    trajectory_path.parent.mkdir(parents=True, exist_ok=True)
    env = simulation_environment()
    if args.scene == "forest":
        env.update({
            "FOREST_TERRAIN_SCENE": str(
                PROJECT / "forest_terrain_3d/generated/basic/environment.usda"
            ),
            "FOREST_NAV_MAP": str(
                PROJECT / "terrain_framework_baseline/results/basic/terrain_costmap.npz"
            ),
            "FOREST_NAV_SENSOR_MESH": str(
                ROOT / "results/basic/sensor_obstacles.npz"
            ),
        })
        scope = "basic forest spawn corridor; no Nav2 involved"
    else:
        free_map = Path("/tmp/forest_nav_open_plane_costmap.npz")
        make_open_plane_map(free_map)
        env["FOREST_NAV_MAP"] = str(free_map)
        scope = "open plane; no Nav2 involved"
    env.update({
        "FOREST_NAV_STATE_PORT": str(args.state_port),
        "FOREST_NAV_COMMAND_PORT": str(args.command_port),
    })
    log_path = ROOT / "results/speed_response_policy.log"
    log = log_path.open("w")
    policy = subprocess.Popen(
        ["bash", "start_g1_policy.sh", "--visualizer", "none"],
        cwd=PROJECT / "nav2_path_tracking",
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    report = {
        "passed": False,
        "backend": f"{args.scene} + existing G1 ONNX + direct local command",
        "session": None,
        "profile": args.profile,
        "phases": [],
        "scene": args.scene,
        "scope": f"{scope}; not a navigation acceptance result",
    }
    probe = None
    try:
        probe = Probe(args.state_port, args.command_port)
        wait_for_ready(probe, policy, args.startup_timeout)
        report["session"] = probe.session
        basic_phases = [
            ("forward_0.12", (0.12, 0.0, 0.0)),
            ("forward_0.20", (0.20, 0.0, 0.0)),
            ("forward_0.30", (0.30, 0.0, 0.0)),
            ("forward_0.45", (0.45, 0.0, 0.0)),
            ("turn_0.15", (0.0, 0.0, 0.15)),
            ("turn_0.25", (0.0, 0.0, 0.25)),
            ("turn_0.50", (0.0, 0.0, 0.50)),
        ]
        def alternating(angular, forward=0.096):
            def command_profile(elapsed, _phase_index):
                sign = 1.0 if int(elapsed / 0.5) % 2 == 0 else -1.0
                return (forward, 0.0, sign * angular)
            return command_profile

        mixed_phases = [
            ("forward_0.096_control", (0.096, 0.0, 0.0)),
            ("mixed_positive_0.20", (0.096, 0.0, 0.20)),
            ("mixed_negative_0.20", (0.096, 0.0, -0.20)),
            ("mixed_alternating_0.20", alternating(0.20)),
            ("mixed_alternating_0.35", alternating(0.35)),
        ]
        threshold_phases = [
            ("forward_0.12_control", (0.12, 0.0, 0.0)),
            ("threshold_negative_0.20", (0.12, 0.0, -0.20)),
            ("threshold_positive_0.20", (0.12, 0.0, 0.20)),
            ("threshold_alternating_0.20", alternating(0.20, 0.12)),
            ("threshold_alternating_0.35", alternating(0.35, 0.12)),
        ]
        sign_phases = [
            ("turn_negative_0.15", (0.0, 0.0, -0.15)),
            ("turn_negative_0.25", (0.0, 0.0, -0.25)),
            ("turn_negative_0.50", (0.0, 0.0, -0.50)),
            ("turn_positive_0.25_control", (0.0, 0.0, 0.25)),
        ]
        angular_threshold_phases = [
            ("turn_negative_0.30", (0.0, 0.0, -0.30)),
            ("turn_negative_0.35", (0.0, 0.0, -0.35)),
            ("turn_negative_0.40", (0.0, 0.0, -0.40)),
            ("turn_positive_0.35", (0.0, 0.0, 0.35)),
        ]
        phases = {
            "basic": basic_phases,
            "mixed": mixed_phases,
            "threshold": threshold_phases,
            "signs": sign_phases,
            "angular-threshold": angular_threshold_phases,
            "all": (basic_phases + mixed_phases + threshold_phases
                    + sign_phases + angular_threshold_phases),
        }[args.profile]
        for phase_index, (name, command_profile) in enumerate(phases):
            samples, start_sim, end_sim = hold_command(
                probe, policy, command_profile, args.phase_seconds, phase_index
            )
            summary_command = (
                (
                    command_profile(0.0, phase_index)[0],
                    0.0,
                    abs(command_profile(0.0, phase_index)[2]),
                )
                if callable(command_profile) else command_profile
            )
            summary = summarize_phase(
                name, summary_command, samples, start_sim, end_sim
            )
            report["phases"].append(summary)
            print(json.dumps(summary, ensure_ascii=False), flush=True)
            stop_index = len(phases) + phase_index
            hold_command(
                probe, policy, (0.0, 0.0, 0.0), 0.8, stop_index
            )
        report["passed"] = all(
            len(phase_samples) > 0 for phase_samples in [probe.samples]
        ) and len(report["phases"]) == len(phases)
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if probe is not None:
            probe.close()
        if policy.poll() is None:
            os.killpg(policy.pid, signal.SIGINT)
        try:
            policy.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(policy.pid, signal.SIGKILL)
            policy.wait()
        log.close()
        output_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n"
        )
        if probe is not None:
            trajectory_path.write_text(
                json.dumps(probe.samples, ensure_ascii=False) + "\n"
            )
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
