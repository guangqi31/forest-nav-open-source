cmake_minimum_required(VERSION 3.8)
project(forest_navigation_bringup)
find_package(ament_cmake REQUIRED)
# 本包只有接线和配置，不编译自写规划或控制节点。
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
