#!/usr/bin/env bash
# 一键打开 Isaac 仿真、G1 策略和中文坐标面板；机器人等待用户输入目标。
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ROOT/start_navigation.sh" --controller "$@"
