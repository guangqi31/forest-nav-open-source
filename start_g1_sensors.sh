#!/usr/bin/env bash
set -uo pipefail
# 只管理本次用户启动的G1、传感器及FAST-LIO进程，不按导航/摔倒状态结束它们。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "${ROOT}/log/g1_sensors"
# 文件锁由本次启动及其子进程共同持有。再次执行复用正在运行的实例，
# 不重复创建传感器发布者、点云转换器或FAST-LIO，也不终止现有仿真。
exec 9>"${ROOT}/log/g1_sensors/launch.lock"
if ! flock -n 9; then
    echo "G1传感器入口已在运行，继续使用现有实例。"
    exit 0
fi
ros_group=""
sim_group=""
stop_by_user() {
    # 只有用户Ctrl+C或显式发送停止信号才清理本次创建的两个进程组。
    # 不挂EXIT陷阱，不因某个子进程失败就连带关闭其他进程。
    trap '' INT TERM
    if [[ -n "${sim_group}" ]]; then kill -TERM -- "-${sim_group}" 2>/dev/null || true; fi
    if [[ -n "${ros_group}" ]]; then kill -TERM -- "-${ros_group}" 2>/dev/null || true; fi
    wait || true
}
trap stop_by_user INT TERM
# ROS环境只进入独立子进程，不向Isaac进程混入系统ROS的Python路径。
setsid bash -c '
 set -e
 source /opt/ros/humble/setup.bash
 source "$1/install/setup.bash"
 exec ros2 launch forest_sensor_bridge sensor_bridge.launch.py sensors_config:="$1/src/forest_sensor_bridge/config/sensors.yaml" fast_lio_config:="$1/src/forest_sensor_bridge/config/fast_lio_sim.yaml" transport_profile:="$1/src/forest_sensor_bridge/config/fastdds_large_points.xml"
' _ "${ROOT}" > "${ROOT}/log/g1_sensors/fast_lio.log" 2>&1 &
ros_group=$!
setsid "${ROOT}/start_g1_ros.sh" "$@" &
sim_group=$!
# 等待两个子进程：其中一个失败时，不向另一个发送终止信号，
# 不根据速度、任务时间、定位、摔倒或导航结果退出/重置仿真。
wait
