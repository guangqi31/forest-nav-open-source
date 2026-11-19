#!/usr/bin/env bash
# 持续查看任务状态、故障原因、当前坐标与重规划次数；Ctrl+C 退出查看。
set -e
source /opt/ros/humble/setup.bash
exec ros2 topic echo /navigation/task_status std_msgs/msg/String
