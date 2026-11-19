#!/usr/bin/env bash
# 只打开坐标控制面板，用于已运行的导航与仿真。
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/forest_nav_tracking_ros_logs}"
cd "$ROOT"
exec /usr/bin/python3 -m forest_nav_control.coordinate_controller
