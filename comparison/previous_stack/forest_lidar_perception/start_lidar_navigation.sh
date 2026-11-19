#!/usr/bin/env bash
# 激光在线地图模式：G1 + 雷达 + CMU地形分析 + 代价地图 + Nav2 + 坐标面板。
set -eo pipefail
LIDAR_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$LIDAR_ROOT/../forest_navigation/start_forest_navigation.sh" --map-mode lidar "$@"
