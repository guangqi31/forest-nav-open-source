#!/usr/bin/env bash
# 独立启动真实G1林地验收；场景结果按时间归档，避免覆盖历史证据。
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
exec /usr/bin/python3 "$ROOT/test_field_navigation.py" "$@"
