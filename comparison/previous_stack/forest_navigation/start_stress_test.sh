#!/usr/bin/env bash
# 压力场景复用正式入口，规划器、控制器与默认面板行为保持一致。
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ROOT/start_forest_navigation.sh" --preset stress "$@"
