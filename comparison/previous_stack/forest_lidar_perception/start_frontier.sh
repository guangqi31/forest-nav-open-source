#!/usr/bin/env bash
set -eo pipefail
FRONTIER_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$FRONTIER_ROOT/install_frontier/setup.bash"
exec ros2 run forest_frontier_bridge frontier_bridge --ros-args -p use_sim_time:=true
