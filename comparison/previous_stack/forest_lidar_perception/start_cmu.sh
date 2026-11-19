#!/usr/bin/env bash
set -eo pipefail
CMU_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$CMU_ROOT/install/setup.bash"
exec ros2 run terrain_analysis terrainAnalysis --ros-args --params-file "$CMU_ROOT/config/terrain.yaml" -r /registered_scan:=/lidar/points_map -r /state_estimation:=/lidar/ground_truth_odometry
