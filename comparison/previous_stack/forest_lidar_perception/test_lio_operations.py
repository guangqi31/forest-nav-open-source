"""真实 LIO/G1 行走中的取消、制动和目标替换测试。"""
import argparse
from datetime import datetime
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import psutil
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger


ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent
sys.path.insert(0, str(PROJECT / "nav2_path_tracking"))
from forest_nav_control.task_client import TaskClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preset", choices=("basic", "complex"), default="basic")
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout 必须为正数")

    output = ROOT / "results" / (
        "lio_operations_" + datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    output.mkdir(parents=True, exist_ok=False)
    mode_env = {
        "FOREST_NAV_MAP_MODE": "lidar",
        "FOREST_NAV_LOCALIZATION": "lio",
        "ROS_DOMAIN_ID": "219",
        "ROS_LOG_DIR": "/tmp/forest_lio_operations_ros",
        "FOREST_NAV_RUN_DIR": str(output / "startup"),
        "FOREST_NAV_MISSION_RESULTS": str(output / "missions"),
        "FOREST_NAV_STATE_PORT": "15761",
        "FOREST_NAV_COMMAND_PORT": "15762",
    }
    os.environ.update(mode_env)
    env = dict(os.environ)
    log = (output / "navigation.log").open("w")
    launcher = subprocess.Popen(
        [
            "bash", str(PROJECT / "forest_navigation/start_forest_navigation.sh"),
            "--preset", args.preset, "--localization", "lio",
            "--controller", "--headless",
        ],
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    node = None
    report = {
        "passed": False,
        "backend": "real G1 + FAST-LIO + online CMU map + Nav2",
        "preset": args.preset,
        "tests": {},
    }
    rclpy.init()
    try:
        node = TaskClient()
        exploration = []
        node.create_subscription(
            String, "/navigation/exploration_status",
            lambda msg: exploration.append(json.loads(msg.data)), 20)
        node.wait_ready(240)

        def acquire_motion(candidates):
            for goal in candidates:
                try:
                    candidate = node.submit(goal)
                except Exception:
                    continue
                start_pose = node.samples[-1][1:3]
                peak = 0.0
                deadline = time.monotonic() + 35
                while time.monotonic() < deadline:
                    node.spin_for(0.1)
                    peak = max(
                        peak,
                        math.dist(node.samples[-1][1:3], start_pose),
                    )
                    if peak > 0.20:
                        return candidate, start_pose, peak, goal
                    if candidate._forest_result_future.done():
                        break
                if not candidate._forest_result_future.done():
                    node.future(candidate.cancel_goal_async(), 5)
                    node.future(candidate._forest_result_future, 10)
            raise TimeoutError("多个短目标均未建立实际运动")

        # 1. 真实运动后统一取消，并检查适配后指令归零与实际制动。
        handle, start, motion_before_cancel, cancel_goal = acquire_motion([
            [1.5, 0.0, 0.0],
            [0.0, 1.5, 0.0],
            [-1.5, 0.0, 0.0],
            [2.0, 0.0, 0.0],
        ])
        cancel = node.create_client(Trigger, "/navigation/cancel")
        if not cancel.wait_for_service(timeout_sec=5):
            raise TimeoutError("/navigation/cancel 服务不可用")
        if not node.future(cancel.call_async(Trigger.Request()), 5).success:
            raise AssertionError("统一取消请求失败")
        result = node.future(handle._forest_result_future, 30)
        if result.status not in (5, 6):
            raise AssertionError(f"取消后Action状态异常：{result.status}")
        cancel_sim_time = node.samples[-1][0]
        deadline = time.monotonic() + 30
        while node.samples[-1][0] - cancel_sim_time < 2.0:
            node.spin_for(0.1)
            if time.monotonic() > deadline:
                raise TimeoutError("取消后没有获得足够仿真时间")
        window = [s for s in node.samples if s[0] >= node.samples[-1][0] - 0.5]
        drift = math.dist(window[0][1:3], window[-1][1:3])
        recent_commands = [
            c for c in node.adapted_commands if time.monotonic() - c[0] < 0.3
        ]
        zero_command = bool(
            recent_commands
            and all(max(abs(v) for v in c[1:]) < 1e-6 for c in recent_commands)
        )
        if not zero_command or drift >= 0.12:
            raise AssertionError(
                f"取消后未稳定停止：zero={zero_command}, drift={drift}")
        report["tests"]["cancel_and_brake"] = {
            "passed": True,
            "action_status": result.status,
            "goal": cancel_goal,
            "motion_before_cancel_m": motion_before_cancel,
            "cancel_settle_sim_sec": node.samples[-1][0] - cancel_sim_time,
            "last_half_second_drift_m": drift,
            "adapted_command_zero": zero_command,
        }

        # 2. 第一目标运动中提交第二目标，旧任务必须终止，新任务必须完成。
        first = node.submit([2.0, 0.0, 0.0])
        first_start = node.samples[-1][1:3]
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            node.spin_for(0.1)
            if math.dist(node.samples[-1][1:3], first_start) > 0.15:
                break
            if first._forest_result_future.done():
                break
        current = node.samples[-1]
        second_goal = [float(current[1] + 0.8), float(current[2]), 0.0]
        second = node.submit(second_goal)
        first_result = node.future(first._forest_result_future, 30)
        if first_result.status not in (5, 6):
            raise AssertionError(f"替换后旧任务未终止：{first_result.status}")
        second_result = node.future(second._forest_result_future, args.timeout)
        if second_result.status != 4:
            raise AssertionError(f"替换后的新任务未成功：{second_result.status}")
        report["tests"]["replace_in_motion"] = {
            "passed": True,
            "old_action_status": first_result.status,
            "new_action_status": second_result.status,
            "new_goal": second_goal,
        }

        # 3. 行走中暂停 FAST-LIO，验证定位断流停止控制，恢复后旧任务不续跑。
        children = psutil.Process(launcher.pid).children(recursive=True)
        lio = next(
            (process for process in children
             if any("fastlio_mapping" in part for part in process.cmdline())),
            None,
        )
        if lio is None:
            raise RuntimeError("没有找到 FAST-LIO 子进程")
        moving = node.submit([2.5, 0.0, 0.0])
        before = node.samples[-1][1:3]
        motion_before_dropout = 0.0
        deadline = time.monotonic() + 45
        moved = False
        while time.monotonic() < deadline:
            node.spin_for(0.1)
            motion_before_dropout = max(
                motion_before_dropout,
                math.dist(node.samples[-1][1:3], before),
            )
            if motion_before_dropout > 0.15:
                moved = True
                break
            if moving._forest_result_future.done():
                break
        if not moved:
            raise AssertionError("LIO断流测试未建立实际运动")
        fault_event_index = len(exploration)
        lio.suspend()
        try:
            result = node.future(moving._forest_result_future, 30)
            if result.status not in (5, 6):
                raise AssertionError(f"LIO断流未终止旧任务：{result.status}")
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                node.spin_for(0.1)
            recent_commands = [
                c for c in node.adapted_commands
                if time.monotonic() - c[0] < 0.3
            ]
            zero_command = bool(
                recent_commands
                and all(max(abs(v) for v in c[1:]) < 1e-6
                        for c in recent_commands)
            )
            if not zero_command:
                raise AssertionError("LIO断流后适配指令未归零")
        finally:
            lio.resume()
        recovery_sim = node.samples[-1][0]
        deadline = time.monotonic() + 30
        while node.samples[-1][0] - recovery_sim < 0.8:
            node.spin_for(0.1)
            if time.monotonic() > deadline:
                raise TimeoutError("LIO恢复后定位未继续更新")
        stable_pose = node.samples[-1][1:3]
        stable_sim = node.samples[-1][0]
        deadline = time.monotonic() + 20
        while node.samples[-1][0] - stable_sim < 1.2:
            node.spin_for(0.1)
            if time.monotonic() > deadline:
                raise TimeoutError("LIO恢复后稳定窗口不完整")
        resumed_drift = math.dist(stable_pose, node.samples[-1][1:3])
        fault_offset = next(
            (index for index, event in enumerate(exploration[fault_event_index:],
                                                start=fault_event_index)
             if event.get("phase") == "fault"),
            None,
        )
        if fault_offset is None:
            raise AssertionError("LIO断流后没有观察到fault阶段")
        post_fault_phases = [
            event.get("phase") for event in exploration[fault_offset:]
            if event.get("phase")
        ]
        if any(phase in ("navigating", "exploring", "arrived")
               for phase in post_fault_phases):
            raise AssertionError(f"LIO恢复后旧任务重新进入运动阶段：{post_fault_phases}")
        if resumed_drift >= 0.12:
            raise AssertionError(f"LIO恢复稳定后仍持续移动：{resumed_drift}")
        report["tests"]["lio_dropout"] = {
            "passed": True,
            "action_status": result.status,
            "motion_before_dropout_m": motion_before_dropout,
            "adapted_command_zero_after_dropout": zero_command,
            "resume_without_old_task_drift_m": resumed_drift,
            "post_fault_phases": post_fault_phases,
        }
        report["passed"] = True
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if node is not None:
            (output / "trajectory.json").write_text(
                json.dumps(node.samples, ensure_ascii=False) + "\n")
            (output / "commands.json").write_text(
                json.dumps({"raw": node.raw_commands,
                            "adapted": node.adapted_commands},
                           ensure_ascii=False) + "\n")
            node.destroy_node()
        rclpy.try_shutdown()
        (output / "verification.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        if launcher.poll() is None:
            os.killpg(launcher.pid, signal.SIGINT)
        try:
            launcher.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(launcher.pid, signal.SIGKILL)
            launcher.wait()
        log.close()
    print(json.dumps(dict(report=report, output=str(output)),
                     ensure_ascii=False, indent=2), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
