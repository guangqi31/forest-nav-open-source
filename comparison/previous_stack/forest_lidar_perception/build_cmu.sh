#!/usr/bin/env bash
# 只构建官方地形分析包；不安装到系统，也不构建CMU规划器和仿真器。
set -eo pipefail
CMU_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
cd "$CMU_ROOT"
colcon --log-base log build --base-paths third_party/cmu_autonomy/src/terrain_analysis third_party/cmu_autonomy/src/terrain_analysis_ext --packages-select terrain_analysis terrain_analysis_ext --build-base build --install-base install --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
