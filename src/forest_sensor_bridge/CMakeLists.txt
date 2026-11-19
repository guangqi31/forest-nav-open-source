cmake_minimum_required(VERSION 3.8)
project(forest_sensor_bridge)
find_package(ament_cmake REQUIRED)
# ROS 节点使用系统 Python；isaac 目录只安装为数据，由 Isaac 自带 Python 导入。
install(PROGRAMS scripts/pointcloud_fields.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch isaac DESTINATION share/${PROJECT_NAME})
ament_package()
