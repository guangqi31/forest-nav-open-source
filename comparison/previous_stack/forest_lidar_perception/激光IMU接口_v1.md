# 激光、IMU与时间同步接口

2026-11-13，第一个传感器接入阶段已完成；没有切换导航定位来源或运行SLAM。

## 可直接订阅的话题

| 话题 | 类型 | 内容 |
|---|---|---|
| /lidar/points | sensor_msgs/PointCloud2 | 原有5Hz仿真采样XYZI，lidar_link坐标，保持兼容 |
| /lidar/points_synced | sensor_msgs/PointCloud2 | 给后续激光惯性算法的入口；XYZI、ring:uint16、time:float32，已确认完整IMU时间覆盖 |
| /imu/data_raw | sensor_msgs/Imu | 200Hz真实物理步采样，imu_link坐标；角速度rad/s，加速度m/s² |
| /sensors/sync_status | std_msgs/String，JSON v1 | 会话、时钟代号、ready、累计样本/同步帧数、缺样、丢帧及数据年龄 |
| /clock | rosgraph_msgs/Clock | 原有同源仿真时钟，整套运行只保留一个发布源 |

/lidar/points_map和/lidar/ground_truth_odometry仍供原CMU/导航使用。未来SLAM应订阅points_synced和imu/data_raw，不应把真值配准点云或真值里程计当成估计结果。

## 采样与时间规则

- G1物理步0.005秒、控制步0.02秒；IMU通过IsaacLab原生ImuCfg和公开的物理子步记录接口逐步采样，使用求解器的角速度与比力，不是把50Hz机体状态插值成200Hz。没有改框架核心或策略。
- 同一采集时间域：1秒起点＋实际物理步数×0.005秒。IMU保存整数纳秒，连续间隔5000000纳秒；雷达每40个物理步采样一次。墙钟只用于失联检查，不填入测量header。
- 控制步末批量交接IMU，保留3仿真秒滚动历史。ROS端按序号去重并检查采样连续性，跨控制步轮询不会只拿到最后一个IMU样本。
- 同步点云只在上一同步帧到本帧的IMU连续、且IMU已覆盖本帧时间终点后发布；不是每帧点云只配一个IMU，也不做近似时间匹配来丢掉高频数据。
- 发现跨包缺样后，停止同步点云并锁定imu_stream_gap_requires_restart，须重启传感器会话。单次缓冲读取异常保留最后有效时间，不伪造测量。会话/时钟代号变化会清理旧缓存，后续估计器也必须据此重置。
- 整体ready同时检查IMU新鲜度和激光新鲜度；暂停、激光停更不能因另一条数据还活着而保持ready。

200Hz指仿真时间采样率。仿真慢于实时、或ROS批量发送时，ros2 topic hz看到的墙钟到达频率可能较低；应检查header间隔5毫秒与连续性。

## 坐标、重力和外参

两传感器固定安装在同一个torso刚体，轴向为机体右手系（X前、Y左、Z上）。当前IMU位于torso原点，激光位于其上方0.65米。

T_imu_lidar的定义是：p_imu = R_imu_lidar × p_lidar + t_imu_lidar。当前R=单位阵、t=(0,0,0.65)米，通过/tf_static发布imu_link→lidar_link。

IMU加速度为包含重力贡献的比力：静止水平时Z约+9.81，不应在入口重复减重力。orientation_covariance[0]=-1，表示不提供姿态估计；角速度和加速度协方差为0表示未知，没有假装成已标定的完美传感器。

为保持现有导航，真值TF由原来的map→lidar_link调整为等价的map→imu_link→lidar_link。map→imu_link仍为完整6DoF真值，固定外参相乘后得到原激光位姿；现有平面odom→base_link分支保留。不能把imu_link静态挂在平面base_link下而丢失机体滚转/俯仰。后续切换SLAM时须接管这条真值TF的发布权，本阶段未做切换。

## 配置和实现

- config/imu.json：频率、安装位置/旋转、滚动缓存长度和协方差约定。
- config/lidar.json：原激光配置，频率5Hz、32线。
- nav2_path_tracking/forest_nav_control/imu_sensor.py：原生IMU配置、逐物理步采样和Kit侧缓存。
- imu_bridge.py：ROS原始IMU、固定外参与时间覆盖门控。
- sensor_contract.py：两种Python环境共享的外参和数据格式校验。
- 传感器目录内imu_buffer.npz属于内部传输，外部算法使用ROS话题。

修改频率或安装位置后需要重启整套传感器；请求的IMU周期必须是物理步长的整数倍，不会为凑频率修改策略或物理步长。

## 启动和检查

原一键导航入口会自动带上IMU。只查看传感器、暂不启动Nav2或发送目标：

```bash
/home/ubuntu/robot_projects/forest_nav/forest_lidar_perception/start_lidar.sh
```

有时限的独立检查（G1保持零目标速度）：

```bash
/home/ubuntu/robot_projects/forest_nav/forest_lidar_perception/start_lidar.sh --check --timeout 120
```

独立模式ROS_DOMAIN_ID默认191，原全导航模式默认186，另一个终端查看时要使用相同值。

## 验证与边界

真实Isaac+G1记录：results/20261113_183751/。接收1000个连续IMU样本、25帧同步点云，时间间隔5毫秒、无缺样，每帧点云时间都有IMU样本覆盖。固定外参检查通过；最后1仿真秒平均比力约(-0.170,-0.004,9.809)m/s²。状态消息和测量消息异步接收，保存的状态计数快照可稍早于最后收到的样本。

28项接口与既有地图/任务保护回归通过；之后补充整数纳秒和激光停更状态检查，相关3项通过。汇总为results/lidar_imu_stage1/verification.json。

这是理想瞬时几何扫描：同帧所有点同时测量，time全部为0，没有伪造旋转扫描的点内时间；ring按真实射线垂直通道保留。尚未加入IMU噪声/偏置、逐束运动畸变和真机时间同步模型。后续选择LIO框架时，需要明确使用瞬时扫描输入或关闭不适用的去畸变假设，不能把当前入口称为已完成SLAM。
