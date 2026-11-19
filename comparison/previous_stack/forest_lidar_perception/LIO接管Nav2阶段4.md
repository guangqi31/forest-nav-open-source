# 第4步：LIO正式接管Nav2定位与地图

2026-11-13。默认森林导航入口已切换到LIO。真实G1短程闭环成功；没有训练、没有修改原策略或IsaacLab/InstinctLab核心。

## 一键启动

```bash
/home/ubuntu/robot_projects/forest_nav/forest_navigation/start_forest_navigation.sh
```

自动启动森林、G1策略、激光/IMU、FAST-LIO、robot_localization、CMU地形图、Nav2和坐标面板。初始化时保持零目标速度，地图/定位/策略/规划器全部就绪后才接受导航。

直接执行一个目标：

```bash
/home/ubuntu/robot_projects/forest_nav/forest_navigation/start_forest_navigation.sh --goal 2 0 0
```

坐标是本次LIO地图的map坐标，原点/轴向来自初始化，不再默认是USD绝对世界坐标。CLI偏航角用弧度，现有面板仍用度。默认打开Isaac，--headless才不显示窗口。

明确选择旧真值导航供对比：

```bash
/home/ubuntu/robot_projects/forest_nav/forest_navigation/start_forest_navigation.sh --localization truth
```

这是启动时互斥选择，LIO失效不会自动回退真值。offline/online旧地图模式默认仍选truth，不能与lio混搭。start_lio_mapping.sh保留第2/3步独立建图/旁路验收用途，不等于这个完整导航入口。

## 实际数据链路

原始激光+IMU → FAST-LIO位姿/配准点云 → 独立CMU → 可扩展地形代价图 → Nav2 Smac2D/RotationShim+DWB → 既有速度适配器 → G1原ONNX策略。

FAST-LIO的控制根平面位姿同时送给现成robot_localization EKF，输出高频/odom及odom→base_link。它只融合LIO位姿，不重复融合原始IMU，也不读取真值速度/位姿。使用本机robot_localization 3.5.4。

最初直接把5Hz测量时间用于控制TF，实际测试出现Nav2查询当前时刻时的外推错误。现由EKF以50Hz仿真时间频率预测到当前，TF有效期偏移0.05秒；map→odom是两次显式坐标修正之间的常量，也使用相同有效期。保留原监视器的未来容差，没有为通过测试放宽它。

定位ready仍独立检查原始LIO测量年龄、LIO/地形状态、滤波输出与地图快照。即使EKF继续预测，LIO停止也会撤销控制许可；不是不断给旧测量换新时间戳。

## 坐标与发布权

本地URDF根已核对为torso_link，策略速度也作用于这个根；IMU安装在同一刚体。通过config/imu.json的安装外参恢复根位姿，再投影为水平base_link；没有误当成pelvis，也不使用世界真值计算控制外参。

导航TF主链：map→odom→base_link→imu_link→lidar_link。

| 输出 | 唯一负责方 |
|---|---|
| /odom、odom→base_link | forest_lio_ekf，robot_localization |
| map→odom、base_link→imu_link | forest_lio_navigation |
| imu_link→lidar_link | 原传感器固定外参发布者 |
| /traversability_map、local图 | forest_lio_navigation，来自独立LIO地图 |
| /navigation/mapping_status | forest_lio_navigation，source=cmu_lio |
| /navigation/bridge_status.pose | 从LIO导航状态取map位姿，localization_source=fast_lio |

map与lio_map为单位别名；内部lio_map→lio_odom→lio_imu→lio_lidar支路保留诊断用途。连续odom不会随着地图平移/偏航修正而跳变，修正由map→odom表达。

仿真通信桥在LIO模式下停止发布真值odom和定位TF；雷达桥停止发布真值配准点云与map→imu_link。仿真反馈仍提供时钟、策略存活/摔倒与失联检查。真值里程计仅给独立评估和Isaac显示支路，不进入定位、地形或规划控制。

## 地图快照与修正时的任务处理

地图仍为0.2米分辨率，初始32米、按观测扩展到最多256米，未知保持-1。LIO图与原真值CMU/地图进程互斥启动，避免两个发布者覆盖同一标准话题。

新图先在/lio/traversability_map与local话题发布。地图状态带两个指纹，包含几何与实际代价字节；导航边界只有拿到一致的全局/局部快照才转发到Nav2。指纹按ROS序列化精度处理float32分辨率，不会因0.2的浮点表示差异误判。

显式坐标修正使reset_generation递增。导航边界先撤销ready、通知上层停止；原安全监视器取消旧子目标，等待旧动作结束再恢复接受新目标。旧目标不会被悄悄迁移坐标或自动续跑。

完整导航默认ROS域186，修正客户端示例：

```bash
ROS_DOMAIN_ID=186 /home/ubuntu/robot_projects/forest_nav/forest_lidar_perception/correct_lio_map.sh --x 2 --y 3 --yaw-deg 10
```

这是绝对地图坐标修正，不是移动机器人。修正后需要按新map坐标重新提交目标。/mapping/save保存新LIO图；/mapping/reset清空观测图、保留连续定位与修正，并通过新代次停止旧任务。

当前边界外目标仍按既有入口拒绝；本阶段没有加入无限地图或自动回环算法。

## Isaac实时路线显示

导航的map坐标已不同于USD世界。单独的lio_plan_visualization只为画线，使用初始一对估计/真值位姿确定固定world←lio_odom，反向应用当前map修正，将/plan送到/navigation/world_plan。通信桥只将此显示路径发给Isaac绿色曲线。

规划器、控制器、地图均不订阅world_plan；显示用真值不会返回导航。实际验收日志记录了路线随重规划实时更新。

## 本次验证

实际记录：results/lio_navigation_20261113_195607/verification.json。

- 一键入口向(2,0,0)导航，Nav2返回成功，程序正常退出。
- 实际位移约1.719m，路径约1.749m，末次估计位置距目标约0.313m；停车未验收，不将Nav2成功当成精确停稳。
- 25对原始LIO/真值位姿，首对固定SE3对齐后位置RMSE约0.0603m，最大误差0.0939m，终点定位误差0.0629m。
- 短程预先定义门槛为至少15对、RMSE<0.15m、终点定位误差<0.2m。原通用40对门槛不适用于这条2米短程；报告分别保留，未事后放宽误差门槛。
- 实际/odom发布者只有forest_lio_ekf，地图只有forest_lio_navigation，地图source为cmu_lio，桥位姿source为fast_lio；真值配准点云消息为0，TF父节点符合预期。
- 本机Humble Python不提供逐消息TF发布者信息，审计范围是父节点唯一性、实际odom发布者和真值TF禁用回归，不宣称获得了逐条TF发布者证据。

最初一键入口还有前沿组件安装路径检查错误，已按实际colcon非合并安装目录修正。预测TF从0.1调整到0.05秒，解决了原未来容差边缘误停；最终短程记录中没有查询超前错误。

74项ROS侧回归与Isaac Python中的2项原策略命令测试均通过，汇总results/lio_navigation_stage4/verification.json。ROS Python3.10不装Torch，两个策略测试在已有Isaac Python3.12环境运行。

这次验证覆盖短程已知目标和正式定位接管。长距离未知目标、实地地图修正中的运动停车、长期漂移/回环没有在本轮物理测试中验证。最后的状态标识、任务轨迹转map及说明修改只做接口回归，没有再次重复物理仿真。

## 实现位置

- lio_navigation.py：一致地图转发、控制根位姿转换、定位ready与修正停旧任务。
- config/lio_ekf.yaml：现成高频滤波器参数。
- lio_plan_visualization.py：独立Isaac画线支路。
- bridge_node.py、lidar_bridge.py：真值导航发布互斥。
- tracking.launch.py、start_forest_navigation.sh：整套启动与定位模式选择。
- test_lio_navigation.py：独立实际导航评估，真值仅在测试/显示支路使用。


## 2026-11-13补充：较远未知目标闭环测试通过

使用正式一键入口和原G1策略，目标为(1,10,0)，初始地图中目标代价为-1（未知），直线距离约10.05米。系统自行选择前沿子目标、更新地图并到达终点，实际轨迹13.26米；已知栅格由1200增加到5471。中间一次子目标失败后自行换点恢复，未人工插入中间目标。

全程使用FAST-LIO定位及CMU生成的地图，没有回退真值导航。205对同时刻位姿仅按首对固定SE3对齐，位置RMSE为5.36厘米；独立真值评估的最终目标水平误差为5.54厘米。真值仅用于独立评估与显示。此次只扩展测试脚本和保存证据，没有更改生产导航参数或机器人策略。该单场景通过不等于长期漂移、回环或持续停车站稳验收。

结果目录：forest_lidar_perception/results/lio_navigation_20261113_201405/。verification.json为验收数据，unknown_navigation.png为初末地图与轨迹对比，测试结果.md为简要记录。复现：在ROS Humble环境运行 forest_lidar_perception/test_lio_navigation.py --goal 1 10 0 --require-unknown --task-timeout 260；该客户端等待时间不修改生产任务管理器的默认时限。


## 2026-11-13补充：范围外远目标进入有限探索

已统一坐标面板、CLI和未知导航管理器的在线准入：当前地图边界外目标作为探索意图接收，经过已知安全观察点逐步观测扩图；终点真正可观测且满足原代价/路径检查后才交给原生Nav2。不会为了目标填图或清障；原12段、6失败、240秒预算及取消/故障保护不变。离线入口仍拒绝范围外目标。15项针对性检查通过（包含真实ROS Action通信、夹具扩图和预算终止），本轮没有新的Isaac物理运动验收。详见forest_lidar_perception/地图范围外目标接纳.md及results/outside_goal_admission/verification.json。
