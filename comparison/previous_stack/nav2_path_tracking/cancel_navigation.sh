#!/usr/bin/env bash
# 取消当前 NavigateToPose 任务；服务响应只表示已提交，终态查看 task_status。
set -e
source /opt/ros/humble/setup.bash
exec ros2 service call /navigation/cancel std_srvs/srv/Trigger '{}'
