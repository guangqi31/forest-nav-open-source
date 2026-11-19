# 桥接①：Isaac 原生传感器 → FAST-LIO

## 范围与状态

本包只装配原生雷达/IMU/时钟、转换 PointCloud2 字段、启动原生 FAST-LIO。
没有创建机器人、森林、仿真主循环、导航控制器或目标输入界面。
代码和 ROS 接口检查完成；实际 RTX 传感器与 G1 场景尚未联合验证。

## 文件职责

- `isaac/sensor_graph.py`：在 Isaac Python 内调用原生传感器与 ROS 发布节点。
- `scripts/pointcloud_fields.py`：在系统 ROS Python 内转换点云字段和逐点时间。
- `config/sensors.yaml`：两个进程共用的传感器位姿、频率、话题、frame 配置。
- `config/fast_lio_sim.yaml`：基于固定上游 jt32 配置另存的仿真输入参数。
- `launch/sensor_bridge.launch.py`：启动字段转换节点及 FAST-LIO；由安装配置计算外参。
- `package.xml`、`CMakeLists.txt`：标准 ROS 包声明及安装规则。

## 消息接口

| 数据 | 话题 | 类型 | 坐标系 |
|---|---|---|---|
| 原生雷达点云 | `/isaac/lidar/points_raw` | sensor_msgs/msg/PointCloud2 | lidar_link |
| FAST-LIO 输入点云 | `/lidar/points` | sensor_msgs/msg/PointCloud2 | lidar_link |
| 原始 IMU | `/imu/data_raw` | sensor_msgs/msg/Imu | imu_link |
| 仿真时间 | `/clock` | rosgraph_msgs/msg/Clock | 不适用 |

原生点云中的 channel_id 转成 uint16 ring；两个 uint32 时间字合并为 uint64 纳秒，
再转成 double 秒 timestamp。保留坐标与强度，不做滤波、降采样或运动补偿。
点按采样时间稳定排列，帧头使用首点时间，符合当前 FAST-LIO 预处理的起点语义。
空帧不发布；格式不符会报告当前消息，节点继续等待，不生成假数据。

IMU 使用原生物理读数与采样时间，包含加速度计比力，角速度使用 rad/s；
不发布仿真真值姿态为估计姿态。雷达 10Hz；IMU 目标 200Hz。
宿主物理步长应不大于 0.005 秒，本包只报告步长不足，不擅自修改。

## ROS 侧启动

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/robot_projects/forest_nav_open_source/install/setup.bash
ros2 launch forest_sensor_bridge sensor_bridge.launch.py
```

这条命令只打开接收侧。没有仿真发布输入时，FAST-LIO 会等待数据，不会自动生成地图。
只打开字段转换可传 `start_fast_lio:=false`。
安装目录配置由构建时复制；修改源配置后重新构建，或者显式传入：

```bash
ros2 launch forest_sensor_bridge sensor_bridge.launch.py \
  sensors_config:=/home/ubuntu/robot_projects/forest_nav_open_source/src/forest_sensor_bridge/config/sensors.yaml
```

## Isaac 宿主侧调用

在 SimulationApp 初始化、机器人实体创建之后，且开始物理运行之前，由宿主加入：

```python
import runpy
api = runpy.run_path(
    "/home/ubuntu/robot_projects/forest_nav_open_source/src/forest_sensor_bridge/isaac/sensor_graph.py"
)
# body_prim_path 是宿主选定的 G1 躯干刚体绝对路径，不是机器人根 Xform。
# 必须保留 sensor_bridge 引用；不调用新的 SimulationApp，不重置宿主物理世界。
sensor_bridge = api["SensorBridge"](body_prim_path=body_prim_path)
```

该调用尚未加入任何旧启动脚本。宿主继续自己的物理循环即可，接口通过物理步回调采样。
用户主动卸载接口时可调用 `sensor_bridge.detach()`，只解绑发布，不关闭仿真。

两个进程必须使用同一个 ROS_DOMAIN_ID，ROS 侧已启用 use_sim_time。
若宿主已有时钟发布者，配置 publish_clock=false，避免重复发布。
默认安装方案：IMU 在刚体原点，雷达在同一刚体上方 0.25 米，均与刚体同向。
安装外参由同一 sensors.yaml 计算，不能单独手改 FAST-LIO 外参再与传感器配置脱节。
本阶段不发布 map→odom 或定位真值 TF；下一桥接阶段再对接定位输出。

## 依赖和剩余验证

FAST-LIO 为本机固定版本 Hesai ROS2 分支，未改源码；lidar_type=3 只选择该分支的
32线兼容预处理格式，不表示 Isaac 的 XT32 资产是真实 JT32。
雷达使用 NVIDIA 支持的 HESAI_XT32_SD10 资产。本次没有下载资产；
宿主必须能访问该 USD，之后应核对实际 channel_id 范围及逐点时间跨度。

当前工具进程无法与 NVIDIA 驱动通信，未启动 RTX 渲染进行真实点云验证。
静止 IMU 比力、实际 10/200Hz、雷达移动时的时间/坐标语义及 FAST-LIO 建图仍待场景验证。
这些没有通过前，不能将本阶段称为传感器端到端验收通过。
所有检查脚本仅存 /tmp，没有安装进生产包。
