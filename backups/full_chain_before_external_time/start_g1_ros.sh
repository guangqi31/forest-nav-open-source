#!/usr/bin/env bash
set -euo pipefail
# 独立G1速度接口入口；复用本机已存在的模型和依赖，不启动旧导航系统。
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_ROOT="${ISAAC_SIM_ROOT:-/home/ubuntu/isaacsim-6.2.0}"
export HIKING_DATA_ROOT="${HIKING_DATA_ROOT:-/home/ubuntu/instintlab-gpu/work/instinctlab_models/data&model/parkour_motion_reference}"
MODEL_ROOT="${HIKING_MODEL_ROOT:-/home/ubuntu/instintlab-gpu/work/instinctlab_models/data&model/checkpoints/parkour_onboard_preview_stair}"
export INSTINCTLAB_DISABLE_VIRTUAL_OBSTACLES=1
export INSTINCTLAB_MINIMAL_TERRAIN=1
export FOREST_DYNAMIC_SOIL=0
# 原样加载上次的复杂三维林地；不使用预制代价地图。
export G1_FOREST_SCENE="${G1_FOREST_SCENE:-${ROOT}/assets/forest_complex/environment.usda}"
export CUDA_LAUNCH_BLOCKING="${CUDA_LAUNCH_BLOCKING:-0}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${ROOT}/src/forest_sensor_bridge/config/fastdds_large_points.xml"
export PYTHONPATH="${ROOT}/src/forest_g1_velocity_bridge:${SIM_ROOT}/extscache/omni.kit.pip_archive-0.0.0+187cf21b.lx64.cp312/pip_prebundle:${PYTHONPATH:-}"
exec "${SIM_ROOT}/python.sh" --no-ros-env \
 "${ROOT}/src/forest_g1_velocity_bridge/scripts/play_g1_ros.py" \
 --task=Instinct-Parkour-Target-Amp-G1-v0 --load_run="${MODEL_ROOT}" \
 --useonnx --num_envs=1 --device="${HIKING_DEVICE:-cuda:0}" \
 --disable_fabric --visualizer kit \
 --kit_args="--enable isaacsim.core.simulation_manager --enable isaacsim.sensors.experimental.rtx --enable isaacsim.sensors.experimental.physics --enable isaacsim.ros2.bridge" "$@"
