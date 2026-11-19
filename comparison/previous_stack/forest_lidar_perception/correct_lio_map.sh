#!/usr/bin/env bash
# 仅修正独立LIO地图坐标，输入是绝对量；默认ROS域与一键LIO入口一致。
set -eo pipefail
LIO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export PYTHONPATH="$LIO_ROOT/../nav2_path_tracking:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-191}"
exec /usr/bin/python3 -m forest_nav_control.lio_correction "$@"
