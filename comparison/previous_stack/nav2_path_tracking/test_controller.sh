#!/usr/bin/env bash
# 实际操作面板控件进行闭环验收；--backend isaac 验证可见 G1 仿真。
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec 9>/tmp/forest_nav_controller_test.lock
flock -n 9 || { echo '控制面板验收已经运行'; exit 1; }
source /opt/ros/humble/setup.bash
cd "$ROOT"
exec /usr/bin/python3 -m forest_nav_control.test_controller "$@"
