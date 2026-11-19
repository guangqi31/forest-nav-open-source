#!/usr/bin/env bash
set -euo pipefail

# Runs the Hiking/InstinctLab task without the procedural-forest extension.
# Use this once to separate a CUDA/IsaacLab issue from a forest-scene issue.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_ROOT="${ISAAC_SIM_ROOT:-/home/ubuntu/isaacsim-6.2.0}"
DATA_ROOT="${HIKING_DATA_ROOT:-/home/ubuntu/instintlab-gpu/work/instinctlab_models/data&model/parkour_motion_reference}"
MODEL_ROOT="${HIKING_MODEL_ROOT:-/home/ubuntu/instintlab-gpu/work/instinctlab_models/data&model/checkpoints/parkour_onboard_preview_stair}"

export HIKING_DATA_ROOT="${DATA_ROOT}"
export FOREST_DYNAMIC_SOIL="0"
export INSTINCTLAB_DISABLE_VIRTUAL_OBSTACLES="${INSTINCTLAB_DISABLE_VIRTUAL_OBSTACLES:-1}"
export INSTINCTLAB_MINIMAL_TERRAIN="${INSTINCTLAB_MINIMAL_TERRAIN:-1}"
# 正常运行关闭CUDA同步调试，避免每次GPU调用阻塞仿真主线程。
# 仅定位CUDA异常时在启动前显式设置CUDA_LAUNCH_BLOCKING=1。
export CUDA_LAUNCH_BLOCKING="${CUDA_LAUNCH_BLOCKING:-0}"
export PYTHONPATH="${SIM_ROOT}/extscache/omni.kit.pip_archive-0.0.0+187cf21b.lx64.cp312/pip_prebundle:${PYTHONPATH:-}"

EXTRA_ARGS=()
if [[ "${HIKING_KEYBOARD_CONTROL:-0}" == "1" ]]; then
  EXTRA_ARGS+=(--keyboard_control)
fi

# Set CUDA_LAUNCH_BLOCKING=1 from the shell when an exact CUDA call site is needed.
exec "${SIM_ROOT}/python.sh" --no-ros-env \
  "${ROOT}/InstinctLab/source/instinctlab/instinctlab/tasks/parkour/scripts/play.py" \
  --task=Instinct-Parkour-Target-Amp-G1-v0 \
  --load_run="${MODEL_ROOT}" \
  --useonnx \
  --num_envs=1 \
  --device="${HIKING_DEVICE:-cuda:0}" \
  --disable_fabric \
  --visualizer kit \
  "${EXTRA_ARGS[@]}" \
  "$@"
