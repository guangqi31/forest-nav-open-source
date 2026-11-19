# 给后续AI的排查入口

## 任务与约束

目标是在Isaac复杂林地中，用真实仿真激光和IMU建图定位，再由成熟开源导航模块给G1原ONNX策略速度，用户输入坐标后自主到达。用户明确要求：只写适配，不自写规划/控制算法；不擅加速度限制、仲裁、强制对齐、任务否决、超时、摔倒退出或自动重置。失败后保留仿真运行。修改前备份，注释中文，测试代码不混入运行链。参照AGENTS.md。

## 当前实际链路

Isaac三维森林 + G1原行走策略 → 原生RTX HESAI雷达/原生IMU/clock → 自写forest_sensor_bridge → FAST-LIO → CMU loam_interface → terrainAnalysis/Ext → FAR Planner → CMU localPlanner → CMU pathFollower → TwistStamped → 自写forest_g1_velocity_bridge → ONNX策略关节动作 → G1物理。

当前新链路中没有Nav2控制器；旧版comparison目录中存在的Nav2/MPPI/自写控制代码不是当前新链路。不要把别的可视化或旧系统当成本项目。

## 实际入口及依赖

- start_g1_sensors.sh：管理G1/传感器/FAST-LIO进程。
- start_g1_ros.sh：Isaac子进程启动配置。
- src/forest_g1_velocity_bridge/scripts/play_g1_ros.py：当前自写宿主；从旧项目play.py复用main/配置解析/ONNX加载，替换本进程_run_play。
- src/forest_g1_velocity_bridge/scripts/forest_scene.py：林地与策略射线网格配置。
- src/forest_g1_velocity_bridge/forest_g1_velocity_bridge/adapter.py：速度三列原值接入观测。
- src/forest_sensor_bridge/isaac/sensor_graph.py：实际IMU、RTX和ROS图装配。
- src/forest_sensor_bridge/scripts/pointcloud_fields.py：点字段适配。
- src/forest_navigation_bringup：CMU/FAR接口配置和launch。

Isaac Python3.12实际editable包指向原机forest_nav/InstinctLab、forest_nav/IsaacLab-main和forest_nav/instinct_rl。这些实际工作树已复制至runtime_sources，不要只看新宿主就断言底层等同纯净上游。实际原机GPU为RTX5070 Laptop，驱动595.99.02，Ubuntu22.04，Isaac6.2.0。

## 当前症状与已测证据

物理200Hz(dt=0.005)、策略50Hz(decimation=4)。每策略步推进0.02仿真秒；整步约35毫秒，因此实际仅约0.58—0.59倍实时。不是新增20毫秒超时限制，超过仍继续运行。ONNX推理约0.55毫秒。backend约22.42毫秒但包含自写传感器回调约4.06毫秒；不能把这些相加，也不能称为纯GPU内核时间。

已修复过自写Controller.set默认撤销记录、重复policy/critic订阅、推理宿主残留critic/AMP/奖励计算，以及启动残留进程问题。修后仍未实时。不要重复把这些当成未改的当前代码。

具体报告：
- reports/当前自写链路实时性复查.md
- reports/两版自写入口差异复查.md
- reports/重启后自写链路复查.md
- reports/自写桥接性能修复.md
- reports/复杂林地全流程测试.md

重启后仍0.59。只移动同步发布到策略步末，A-B-A整步35.38→35.07→35.67毫秒，无明显改善。临时不创建和发布新版IMU后仍31.72毫秒；该窗口没有有效FAST-LIO定位，不能算有效导航或把全部差值归因IMU。

找到自写启动器未隔离系统ROS Python路径：Isaac3.12进程混入ROS3.10路径。严格只隔离Isaac子进程并选内置ROS库后，完整链路仍约0.589，非当前性能主因；尚未正式修复这个隔离遗漏。不能将Isaac内置库同时注入系统ROS进程，曾导致spdlog符号冲突；只清理ROS但不补原生桥所需库也曾初始化失败。正式入口已恢复，不使用临时试验配置。

同一complex森林USD两版hash相同。旧入口默认basic，新入口默认complex，但不确定用户上次实际选择。旧版历史有0.16/0.19实时率记录，不代表用户后来能用那版也慢，不能用它代替同条件比较。比较用原始历史文件在comparison/evidence。

## 没有完成的验收

1:1实时未达成。真实全链路曾驱动G1、FAR曾报告到达，但出现过冲、反向指令和跌倒，稳定到点/停车验收未通过。未知环境鲁棒性、真机迁移不能称为完成。

报告包含多日期历史状态；sources.lock.json顶层早期checks和旧README也可能过时，应以当前源码及最近报告为准。未定位唯一性能根因，不要将推断写成已证事实。不要为了“实时”扩大dt、跳步、伪造时钟或随意降低策略频率。
