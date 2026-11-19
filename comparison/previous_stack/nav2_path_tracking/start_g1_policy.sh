#!/usr/bin/env bash
# 启动已有 ONNX 策略，通过可选导航入口接收速度；不训练、不改依赖环境。
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT="$(dirname -- "$ROOT")"
export PYTHONPATH="$ROOT:${PYTHONPATH:-}"
export FOREST_NAV_MAP="${FOREST_NAV_MAP:-$PROJECT/forest_costmap_test/forest_costmap.npz}"
exec "$PROJECT/validate_hiking_base.sh" --nav2_control "$@"
