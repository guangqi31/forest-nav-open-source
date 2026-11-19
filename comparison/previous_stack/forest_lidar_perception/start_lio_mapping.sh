#!/usr/bin/env bash
# 一键启动G1、森林、激光/IMU和FAST-LIO；默认保持零速度，--motion-test才发送测试目标。
set -eo pipefail
LIO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export PYTHONPATH="$LIO_ROOT/../nav2_path_tracking:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-191}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/forest_lio_ros}"
exec /usr/bin/python3 "$LIO_ROOT/run_lio.py" "$@"
