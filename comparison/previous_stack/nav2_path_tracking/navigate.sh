#!/usr/bin/env bash
# 给运行中的导航服务下发目标，默认使用实时定位作为起点。
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/forest_nav_tracking_ros_logs}"
cd "$ROOT"
exec /usr/bin/python3 -m forest_nav_control.task_client "$@"
