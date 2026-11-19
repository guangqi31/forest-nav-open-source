#!/usr/bin/env bash
# 接已有激光/IMU话题；不启动第二套仿真、不修改Nav2定位。
set -eo pipefail
LIO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -x "$LIO_ROOT/install_lio/fast_lio/lib/fast_lio/fastlio_mapping" ]]; then
  echo "请先运行 $LIO_ROOT/build_lio.sh"; exit 2
fi
source /opt/ros/humble/setup.bash
source "$LIO_ROOT/install/setup.bash"
source "$LIO_ROOT/install_lio/setup.bash"
export PYTHONPATH="$LIO_ROOT/../nav2_path_tracking:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-191}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/forest_lio_ros}"
# 同一ROS域只允许一组估计器，防止位姿与地图出现多个同名发布者。
exec flock -n "/tmp/forest_lio_${ROS_DOMAIN_ID}.lock" ros2 launch "$LIO_ROOT/lio.launch.py" "$@"
