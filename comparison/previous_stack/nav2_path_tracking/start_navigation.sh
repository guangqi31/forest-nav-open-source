#!/usr/bin/env bash
# 一键启动：Nav2 + 原有 G1 ONNX 策略 + 目标执行；默认目标为前方1米。
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec 9>/tmp/forest_nav_live_navigation.lock
flock -n 9 || { echo "已有一键导航在运行" >&2; exit 1; }
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/forest_nav_tracking_ros_logs}"
cd "$ROOT"
exec /usr/bin/python3 -m forest_nav_control.start_navigation "$@"
