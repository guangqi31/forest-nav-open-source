#!/usr/bin/env bash
set -euo pipefail
# 只启动G1传感器与FAST-LIO，不启动FAR或旧导航。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "${ROOT}/log/g1_sensors"
# ROS Humble节点运行于系统Python环境，与Isaac自带Python分进程。
# 子进程自行加载ROS环境，不向Isaac进程混入ROS Python路径。
bash -c '
 source /opt/ros/humble/setup.bash
 source "$1/install/setup.bash"
 exec ros2 launch forest_sensor_bridge sensor_bridge.launch.py sensors_config:="$1/src/forest_sensor_bridge/config/sensors.yaml" fast_lio_config:="$1/src/forest_sensor_bridge/config/fast_lio_sim.yaml" transport_profile:="$1/src/forest_sensor_bridge/config/fastdds_large_points.xml"
' _ "${ROOT}" > "${ROOT}/log/g1_sensors/fast_lio.log" 2>&1 &
# 不按定位或任务状态结束节点，也不添加超时清理/看门狗。
exec "${ROOT}/start_g1_ros.sh" "$@"
