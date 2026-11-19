# 原生CMU/FAR上层集成

本包只提供配置和启动接线，不含自写算法节点，不修改上游源码。
输入为FAST-LIO的/Odometry、/cloud_registered和已有仿真/录制数据的/clock。
不启动FAST-LIO、G1或CMU vehicle_simulator，避免重复发布定位和控制状态。

## 启动

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/robot_projects/forest_nav_open_source/install/setup.bash
ros2 launch forest_navigation_bringup navigation.launch.py
```

FAR原生RViz配置带GoalpointTool，用它向/goal_point发送map坐标目标。
上层接口检查可用rviz:=false cmd_vel_topic:=/integration_check/cmd_vel，避免驱动已存在的机器人订阅者。
这只是话题重映射，不是新增速度仲裁。

## 接口

/Odometry(camera_init/body)与/cloud_registered(camera_init世界系点云)
→原生loam_interface→/state_estimation(map/sensor)、/registered_scan(map)
→terrainAnalysis/Ext→/terrain_map、/terrain_map_ext
→FAR→/way_point→localPlanner→/path(vehicle)→pathFollower→/cmd_vel(TwistStamped)。
FAST-LIO历史轨迹必须重映射到/lio/path，不能发布到CMU跟踪器的/path。
不能将/cloud_registered_body替换到世界点云接口。

TF：map→camera_init(静态重合)→body(FAST-LIO动态)→sensor(静态重合)→vehicle(原生安装偏移)。
map就是本地建图坐标，不是Isaac世界坐标。目标坐标应从这张地图选取。
IMU参考点与G1躯干原点重合的当前安装方案才能直接使用body→sensor的单位变换。
不重复应用雷达0.25m安装偏移；该偏移已由FAST-LIO外参使用。

## 参数来源与边界

navigation.yaml中CMU参数从固定版本原生launch原值展开。
FAR加载原生default.yaml，仅覆盖use_sim_time、world_frame和is_static_env。
is_static_env=false用于消费在线观测，不会打开自写探索算法。
机器人尺寸、速度、地形阈值是上游基线，尚未做G1适配标定；不能宣称已完成G1导航验收。
FAST-LIO当前Odometry不提供有效twist，不能把其中的零值当作实测速度。
上游自身的减速、停止等算法行为保持原样，没有在本包添加保护、否决、恢复、超时退出。

## 验收层次

先验收坐标、时间、话题和原生节点启动，再检查地形/路点/路径/速度输出。
编译通过和节点启动不等于未知环境导航通过；实际传感器、G1运动及完整闭环须单独记录。
