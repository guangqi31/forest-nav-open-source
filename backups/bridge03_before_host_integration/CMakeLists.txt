cmake_minimum_required(VERSION 3.8)
project(forest_g1_velocity_bridge)
find_package(ament_cmake REQUIRED)
# 由Isaac自带Python导入，不安装到系统ROS Python 3.10的site-packages。
install(DIRECTORY forest_g1_velocity_bridge DESTINATION share/${PROJECT_NAME}/python
        PATTERN "__pycache__" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
