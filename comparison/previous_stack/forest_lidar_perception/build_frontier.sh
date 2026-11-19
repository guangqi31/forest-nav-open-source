#!/usr/bin/env bash
# 单独构建前沿搜索接口；上游 ROS2 explore_lite 搜索源码保持原样。
set -eo pipefail
FRONTIER_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
cd "$FRONTIER_ROOT"
colcon --log-base log_frontier build --base-paths frontier_bridge --build-base build_frontier --install-base install_frontier --cmake-args -DBUILD_TESTING=OFF
