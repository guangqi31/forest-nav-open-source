# 三维林地中的 G1 坐标导航

> 2026-11-13：默认入口已切换为3D激光雷达→CMU→在线代价地图→Nav2→G1。下文早期离线地图和验收记录属于历史配置；旧模式使用`--map-mode offline`。最新使用说明见[激光导航](../forest_lidar_perception/README.md)，短程闭环记录见`../forest_lidar_perception/results/full_integration_20261113/`。

本目录把已有 ONNX 行走策略、真实 Isaac Sim 三维林地、该林地转换出的代价图和 Nav2 接起来。用户在中文面板输入地图坐标，Nav2 规划与计算速度，现有适配器和本机桥接将速度送入 G1 策略；机器人实际状态回传为 `/odom`、`map → odom → base_link` 和仿真时钟。

## 启动

```bash
/home/ubuntu/robot_projects/forest_nav/forest_navigation/start_forest_navigation.sh
```

默认打开 `basic` 三维林地、G1 策略、导航栈和中文坐标面板，等待目标输入。地图坐标单位为米，朝向输入单位为度；新林地中 `map` 与 `world` 重合，出生点约 `(0,0)`。面板默认目标 `(1,0,0°)`，可用来做第一段短程测试。关闭面板会关闭本次一键启动的进程。

也可显式选择预设：

```bash
/home/ubuntu/robot_projects/forest_nav/forest_navigation/start_forest_navigation.sh --preset basic
```

支持 `calibration`、`basic`、`complex`，但本轮真实 G1 导航验收以 `basic` 为准，其他预设仅表示具备加载入口。行走中的 Isaac 视口会把当前实际 `/plan` 绘制为绿色路线；该曲线只用于显示，没有碰撞属性，也不改变 Nav2 或三维落脚规划。

## 一致性与安全范围

- 物理场景来自 `forest_terrain_3d/generated/<preset>/environment.usda`；地图来自同一场景转换的 `terrain_framework_baseline/results/<preset>/terrain_costmap.npz`。启动前检查场景文件哈希与转换时记录一致。
- 策略深度相机读取实际支撑地面、实际碰撞障碍网格和原有的机器人自遮挡对象；纯视觉树冠不会作为碰撞障碍。左右脚扫描器读取支撑地面。运行时会验证深度数值和已知树干的射线命中。传感器专用网格不创建额外物理碰撞。
- 基于实际使用的 G1 URDF 名义姿态，碰撞包络保守外接半径约0.348米；当前 Nav2 采用0.45米圆形足迹，膨胀半径0.9米。运行中以实际刚体姿态抽样记录包络、非脚部接触和地面相对高度；采样结果不代表所有姿态与所有地形都适用。
- 当前速度**指令**上限0.45米/秒；策略控制下机体瞬时实际速度可能高于指令。前期给低层适配器设置的最大允许值0.7米/秒依然是上游安全上限。
- 静态地图是已知场景离线转换结果，尚未实现在线未知环境建图。当前地面及障碍物是刚性碰撞，不模拟松土沉陷或承载力。
- 新林地地图保留未知空间；跌倒判据使用身体相对局部地面高度。场景边缘预留2米作为结束条件。

## 复现测试与记录

```bash
/home/ubuntu/robot_projects/forest_nav/forest_navigation/test_navigation.sh --smoke
/home/ubuntu/robot_projects/forest_nav/forest_navigation/test_navigation.sh
/home/ubuntu/robot_projects/forest_nav/forest_navigation/test_panel.sh
/home/ubuntu/robot_projects/forest_nav/forest_navigation/test_speed_response.sh --scene forest --profile threshold
/home/ubuntu/robot_projects/forest_nav/forest_navigation/test_path_overlay.sh
```

测试使用独立 ROS 域和 UDP 端口。`--smoke` 包含真实G1短程、运动中取消以及后续2个仿真秒停稳检查；完整测试再继续执行 `(6,2,0)` 路线。图形面板测试自动操作真实 Tk 按钮并检查机器人位姿。速度响应测试直接给既有策略发送阶梯指令，区分纯前进、纯转向和混合换向；路径叠加测试要求实际 Nav2 成功、Isaac 生成路线并保存视口截图。

结果在 `results/`：

- `g1_smoke_verification.json`、`g1_navigation_verification.json`：目标结果、误差、真实速度和取消情况。
- `g1_navigation_trajectory.json`、`g1_navigation_plans.json`、`g1_navigation_commands.json`：真实状态、规划路线和速度指令。
- `basic/preflight.json`、`basic/sensor_runtime_verification.json`、`basic/safety_runtime_verification.json`：尺寸、传感器和运动中包络/接触检查。
- `g1_navigation_attempt1_analysis.json`：保留首次长路线接近目标后超时的记录，不以接近目标冒充成功。
- `panel_verification.json`、`panel_forest.png`：图形面板实际按钮测试结果和截图。
- `speed_response_threshold_verification.json`：低速前进与转向阈值标定结果。
- `path_overlay_test.json`、`path_overlay_forest.png`：长目标行走时的实际路线叠加验收和 Isaac 视口截图。

仿真中的超时和停止保护仍需结合实际机器人的硬件急停、传感器与安全控制另行验证。


## 2026-11-12 终点朝向修复

原配置实际朝向容差为0.5rad；现在启用原生 Rotation Shim 终点朝向控制和 PoseProgressChecker。内部停止阈值0.03rad，独立验证到达时和停稳后误差均不超过0.2rad；读取服务端实际参数并归档。正向、反向原地转向、(6,2,0)长目标及取消停车均通过，停稳朝向误差依次0.093/0.147/0.138rad。长目标停稳位置误差约0.307m，位置精度仍需下一阶段验证。

    /home/ubuntu/robot_projects/forest_nav/forest_navigation/test_navigation.sh --heading-check

完整结果和当次配置见 results/heading_success_20261112/。取消后折返路径的振荡记录保存在 results/heading_attempt2/，不包含在本次终点朝向修复的通过范围中。
