#!/usr/bin/env bash
# 只启动G1、独立3D雷达、200Hz原生IMU和ROS同步数据桥，不启动Nav2、不发送运动目标。
set -eo pipefail
LIDAR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export PYTHONPATH="$LIDAR_ROOT/../nav2_path_tracking:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-191}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/forest_lidar_ros}"
exec /usr/bin/python3 "$LIDAR_ROOT/run_sensor.py" "$@"
