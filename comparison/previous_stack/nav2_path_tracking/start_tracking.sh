#!/usr/bin/env bash
# 只启动导航服务，等待真实仿真状态；不会伪造机器人定位。
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/forest_nav_tracking_ros_logs}"
exec ros2 launch "$ROOT/launch/tracking.launch.py" "$@"
