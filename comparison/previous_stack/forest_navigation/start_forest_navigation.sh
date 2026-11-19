#!/usr/bin/env bash
# 默认LIO在线定位/地图，启动真实Nav2、既有ONNX策略和坐标面板；truth需显式指定。
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PRESET=basic
MAP_MODE=lidar
LOCALIZATION=''
PLANNER="${FOREST_NAV_PLANNER:-far}"
NAV_CONTROLLER="${FOREST_NAV_CONTROLLER:-mppi}"
FORWARD_ARGS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --preset) PRESET="$2"; shift 2 ;;
    --map-mode) MAP_MODE="$2"; shift 2 ;;
    --nav-controller) NAV_CONTROLLER="$2"; shift 2 ;;
    --planner) PLANNER="$2"; shift 2 ;;
    --localization) LOCALIZATION="$2"; shift 2 ;;
    *) FORWARD_ARGS+=("$1"); shift ;;
  esac
done
set -- "${FORWARD_ARGS[@]}"
case "$MAP_MODE" in offline|online|lidar) ;; *) echo '地图模式应为offline/online/lidar'; exit 2 ;; esac
case "$NAV_CONTROLLER" in mppi|dwb) ;; *) echo '导航控制器应为mppi/dwb'; exit 2 ;; esac
export FOREST_NAV_CONTROLLER="$NAV_CONTROLLER"
case "$PLANNER" in far|legacy) ;; *) echo '规划方案应为far/legacy'; exit 2 ;; esac
export FOREST_NAV_PLANNER="$PLANNER"
# 用户指定的地形代价上限，终点检查与中间观察点筛选使用同一数值。
export FOREST_NAV_MAX_GOAL_COST="${FOREST_NAV_MAX_GOAL_COST:-40}"
export FOREST_NAV_MAP_MODE="$MAP_MODE"
# 用户指定的仿真直行指令上限；适配器和UDP桥共用此值，转向/停车不提速。
export FOREST_NAV_MAX_FORWARD_SPEED="${FOREST_NAV_MAX_FORWARD_SPEED:-1.8}"
if [[ -z "$LOCALIZATION" ]]; then
  if [[ "$MAP_MODE" == lidar ]]; then LOCALIZATION=lio; else LOCALIZATION=truth; fi
fi
case "$LOCALIZATION" in truth|lio) ;; *) echo '定位来源应为lio/truth'; exit 2 ;; esac
if [[ "$PLANNER" == far ]]; then
  if [[ "$MAP_MODE" != lidar || "$LOCALIZATION" != lio || "$NAV_CONTROLLER" != mppi ]]; then
    echo 'FAR入口使用lidar + LIO + MPPI；旧模式请显式指定--planner legacy'; exit 2
  fi
  if [[ ! -x "$ROOT/../forest_far_navigation/install/far_planner/lib/far_planner/far_planner" ]]; then
    echo 'FAR尚未构建，请运行forest_far_navigation/build_far.sh'; exit 2
  fi
fi
if [[ "$LOCALIZATION" == lio && "$MAP_MODE" != lidar ]]; then echo 'LIO定位必须使用lidar在线地图'; exit 2; fi
export FOREST_NAV_LOCALIZATION="$LOCALIZATION"
if [[ "$LOCALIZATION" == lio && ! -x "$ROOT/../forest_lidar_perception/install_lio/fast_lio/lib/fast_lio/fastlio_mapping" ]]; then
  echo 'LIO尚未构建，请先运行forest_lidar_perception/build_lio.sh'; exit 2
fi
if [[ "$MAP_MODE" == online && ! -x "$ROOT/../forest_online_mapping/build/online_terrain" ]]; then
  echo '在线地图模块尚未构建，请先运行forest_online_mapping/build.sh'; exit 2
fi
if [[ "$MAP_MODE" == lidar && ! -x "$ROOT/../forest_lidar_perception/install/terrain_analysis/lib/terrain_analysis/terrainAnalysis" ]]; then
  echo 'CMU模块尚未构建，请先运行forest_lidar_perception/build_cmu.sh'; exit 2
fi
if [[ "$PLANNER" != far && "$MAP_MODE" == lidar && ! -x "$ROOT/../forest_lidar_perception/install_frontier/forest_frontier_bridge/lib/forest_frontier_bridge/frontier_bridge" ]]; then
  echo '前沿搜索接口尚未构建，请先运行forest_lidar_perception/build_frontier.sh'; exit 2
fi
case "$PRESET" in basic|calibration|complex|stress) ;; *) echo '场景应为 basic/calibration/complex/stress'; exit 2 ;; esac
/usr/bin/python3 "$ROOT/preflight.py" --preset "$PRESET"
export FOREST_TERRAIN_SCENE="$ROOT/../forest_terrain_3d/generated/$PRESET/environment.usda"
export FOREST_NAV_MAP="$ROOT/../terrain_framework_baseline/results/$PRESET/terrain_costmap.npz"
export FOREST_NAV_SENSOR_MESH="$ROOT/results/$PRESET/sensor_obstacles.npz"
export FOREST_NAV_PARAMS="$ROOT/navigation.yaml"
if [[ "$MAP_MODE" == lidar ]]; then
  export FOREST_NAV_PARAMS="$ROOT/../forest_lidar_perception/config/navigation.yaml"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-186}"
export FOREST_NAV_STATE_PORT="${FOREST_NAV_STATE_PORT:-15461}"
export FOREST_NAV_COMMAND_PORT="${FOREST_NAV_COMMAND_PORT:-15462}"
# 只有明确提供--goal时才自动执行目标；其他选项默认仍打开坐标面板。
# 避免沿用旧场景的(-7,-2)默认目标而意外远行。
HAS_TARGET=0
for ARG in "$@"; do
  if [[ "$ARG" == '--goal' || "$ARG" == '--controller' || "$ARG" == '--check-ready' ]]; then HAS_TARGET=1; fi
done
if [[ "$HAS_TARGET" == 0 ]]; then set -- --controller "$@"; fi
exec "$ROOT/../nav2_path_tracking/start_navigation.sh" "$@"
