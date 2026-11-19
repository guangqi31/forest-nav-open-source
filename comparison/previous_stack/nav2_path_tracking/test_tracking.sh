#!/usr/bin/env bash
# 在独立 ROS 域和本机端口测试控制闭环；--backend isaac 切换到真实 G1 策略。
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec 9>/tmp/forest_nav_tracking_test.lock
flock -n 9 || { echo "已有第二阶段测试在运行" >&2; exit 1; }
source /opt/ros/humble/setup.bash
cd "$ROOT"
exec /usr/bin/python3 -m forest_nav_control.test_tracking "$@"
