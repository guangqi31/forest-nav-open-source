#!/usr/bin/env bash
# 独立 ROS 域与端口；防止同时运行多份验收程序互相抢占资源。
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec 9>/tmp/forest_nav_tasks_test.lock
flock -n 9 || { echo '导航任务测试已经在运行'; exit 1; }
source /opt/ros/humble/setup.bash
cd "$ROOT"
exec /usr/bin/python3 -m forest_nav_control.test_tasks
