#!/usr/bin/env bash
# 单独构建上游ROS2 FAST-LIO；不修改Isaac、策略或现有CMU工作区。
set -eo pipefail
LIO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
cd "$LIO_ROOT"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
colcon --log-base log_lio build --base-paths third_party/fast_lio \
  --packages-select fast_lio --build-base build_lio --install-base install_lio \
  --executor sequential --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
