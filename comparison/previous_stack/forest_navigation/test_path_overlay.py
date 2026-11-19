"""验证行走中的 Isaac 视口会显示实际 Nav2 全局路线并保存截图。"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal", nargs="+", type=float,
                        default=[6.0, 2.0, 0.0])
    args = parser.parse_args()
    if len(args.goal) not in (2, 3):
        parser.error("--goal 必须是 x y 或 x y yaw")
    os.environ.update(
        ROS_DOMAIN_ID="189",
        FOREST_NAV_STATE_PORT="15491",
        FOREST_NAV_COMMAND_PORT="15492",
        ROS_LOG_DIR="/tmp/forest_path_overlay_logs",
    )
    from preflight import prepare

    prepare("basic")
    capture = ROOT / "results/path_overlay_forest.png"
    capture.parent.mkdir(parents=True, exist_ok=True)
    if capture.exists():
        capture.unlink()
    os.environ.update({
        "FOREST_TERRAIN_SCENE": str(
            PROJECT / "forest_terrain_3d/generated/basic/environment.usda"
        ),
        "FOREST_NAV_MAP": str(
            PROJECT / "terrain_framework_baseline/results/basic/terrain_costmap.npz"
        ),
        "FOREST_NAV_SENSOR_MESH": str(
            ROOT / "results/basic/sensor_obstacles.npz"
        ),
        "FOREST_NAV_PARAMS": str(ROOT / "navigation.yaml"),
        "FOREST_NAV_PATH_CAPTURE": str(capture),
    })
    sys.path.insert(0, str(PROJECT / "nav2_path_tracking"))
    import rclpy
    from forest_nav_control.start_navigation import simulation_environment
    from forest_nav_control.task_client import TaskClient

    children, logs = [], []
    report = {
        "passed": False,
        "backend": "Isaac Sim Kit + live Nav2 /plan overlay",
        "goal": args.goal,
    }
    node = None

    def start(name, command, env=None):
        log = (ROOT / "results" / f"path_overlay_{name}.log").open("w")
        logs.append(log)
        child = subprocess.Popen(
            command,
            cwd=PROJECT / "nav2_path_tracking",
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        children.append(child)
        return child

    rclpy.init()
    try:
        start("navigation", ["bash", "start_tracking.sh"])
        policy = start(
            "policy",
            ["bash", "start_g1_policy.sh", "--visualizer", "kit"],
            simulation_environment(),
        )
        node = TaskClient()
        node.wait_ready(240)
        outcome = node.wait_task(node.submit(report["goal"]), timeout=180)
        if outcome["action_status"] != 4:
            raise AssertionError(f"导航未成功：{outcome}")
        report["action"] = outcome
        verification_path = ROOT / "results/basic/path_overlay_verification.json"
        deadline = time.monotonic() + 20
        overlay = None
        while time.monotonic() < deadline:
            if policy.poll() is not None:
                raise RuntimeError(f"策略进程提前退出，退出码 {policy.returncode}")
            if verification_path.exists():
                try:
                    candidate = json.loads(verification_path.read_text())
                except (ValueError, OSError):
                    candidate = None
                if (isinstance(candidate, dict) and candidate.get("passed")
                        and candidate.get("display_points", 0) > 0
                        and capture.exists() and capture.stat().st_size > 0):
                    overlay = candidate
                    break
            node.spin_for(0.1)
        if overlay is None:
            raise TimeoutError("未收到已绘制的 Nav2 路线或其视口截图")
        report.update(
            passed=True,
            overlay=overlay,
            captured_image=str(capture),
            capture_bytes=capture.stat().st_size,
            final_pose=list(node.samples[-1][1:4]),
            position_error_m=math.dist(node.samples[-1][1:3], report["goal"][:2]),
        )
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        (ROOT / "results/path_overlay_test.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n"
        )
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        for log in logs:
            log.close()
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
