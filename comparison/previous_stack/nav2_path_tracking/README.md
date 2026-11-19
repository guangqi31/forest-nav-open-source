## 用坐标控制器操作仿真机器人

当前导航最大前进速度为 **0.7m/s**，线加减速度为 **0.8m/s²**。
这是速度上限，转弯和接近终点时会主动减速；实际速度以里程计反馈为准。
此前 0.35m/s 配置的验收记录属于历史结果。提高速度后的 G1 实测见
`results/fast_g1_report.json`。

直接运行：

```bash
/home/ubuntu/robot_projects/forest_nav/nav2_path_tracking/start_navigation_controller.sh
```

一次打开 **Isaac 仿真窗口 + G1 现有策略 + 中文坐标控制面板 + Nav2**。
启动过程中面板显示等待，机器人不自动前往默认目标；出现“已就绪，可以发送目标”后：

1. 输入目标 X、Y（米）和最终朝向（度）。
2. 点击“前往目标 / 替换目标”，在仿真窗口观察机器人自己规划并行走。
3. 到达后继续输入下一个坐标；运动中发送新目标会替换旧目标。
4. 点击“停止导航”取消任务并制动。关闭这次一键启动的面板，会关闭同批启动的仿真和导航。

默认 X=-7、Y=-2、朝向=0，是从当前出生点约 X=-8、Y=-2 向前走约一米的测试目标。
“填入当前位置”“填入前方1米”只填写输入框，必须点击“前往”才会开始导航。
面板使用 map 地图坐标，不是 Isaac world 坐标；当前场景 map=(-8,-2) 对应 world=(0,0)。

如果仿真和导航服务已在运行，只打开面板：

```bash
/home/ubuntu/robot_projects/forest_nav/nav2_path_tracking/open_navigation_controller.sh
```

单独关闭这个附加面板会请求停止当前导航，但不会关闭其他终端启动的仿真。
面板与仿真必须使用同一个 ROS_DOMAIN_ID；默认普通启动不需要额外设置。

实现位于 `forest_nav_control/coordinate_controller.py`：普通 ROS 客户端直接调用
`/navigate_to_pose`，异步处理接收、反馈、替换和结果；当前位置来自实际 `/odom`。
停止通过已有 `/navigation/cancel`，等待和断开状态依据桥接反馈、控制许可及导航生命周期。
旧任务迟到的回调不能覆盖新任务；提交时按停止，会取消迟到才被接受的旧目标。
面板关闭前提交取消请求，一键启动器随后回收自己创建的进程。

面板验收（会实际打开窗口）：

```bash
/home/ubuntu/robot_projects/forest_nav/nav2_path_tracking/test_controller.sh
/home/ubuntu/robot_projects/forest_nav/nav2_path_tracking/test_controller.sh --backend isaac
```

验收直接填写 Tk 输入框、触发按钮，检查实际反馈位置和速度，不绕过界面调用命令行。
结果保存为 `results/controller_kinematic_report.json`、`results/controller_isaac_report.json`，
界面截图保存为 `results/controller_kinematic.png`、`results/controller_isaac.png`。
Isaac 模式只验收短程坐标到达；复杂林地绕障仍需要后续测试。

## 坐标导航任务入口（新增）

一键启动 G1 现有 ONNX 策略、导航服务，并前往地图坐标（米、弧度）：

```bash
cd /home/ubuntu/robot_projects/forest_nav/nav2_path_tracking
./start_navigation.sh --goal -7 -2 0
```

启动后在另一个终端下发新目标；Nav2 替换旧目标，不用重启仿真：

```bash
/home/ubuntu/robot_projects/forest_nav/nav2_path_tracking/navigate.sh --goal -6 -2 0
/home/ubuntu/robot_projects/forest_nav/nav2_path_tracking/navigation_status.sh
/home/ubuntu/robot_projects/forest_nav/nav2_path_tracking/cancel_navigation.sh
```

`--goal x y [yaw]` 使用 map 坐标，省略 yaw 时为 0。当前仿真出生点 map=(-8,-2)，
对应 Isaac world=(0,0)。目标坐标不能直接使用屏幕像素或未转换的世界坐标。
到达后服务保持运行；`--exit-on-goal` 用于验收时到达即退出。后续目标直接调用
`navigate.sh`；同一时刻交给 Nav2 执行最新目标。

新增职责与接口：

| 文件/接口 | 行为 |
|---|---|
| `forest_nav_control/task_client.py` | 客户端提交原生 `/navigate_to_pose` Action，保存实际轨迹和反馈 |
| `config/navigate.xml` | ComputePathToPose → FollowPath，每秒重规划 |
| `config/navigation.yaml` | RotationShim 航向对齐 + 原有 DWB 跟踪，速度限制和碰撞检查保持生效 |
| `/goal_pose` | Nav2 自带的 PoseStamped 目标入口，可由 RViz 发送，frame_id=map |
| `/navigation/cancel` | Trigger 服务，提交取消当前导航的请求，终态由 Action/任务状态确认 |
| `/navigation/task_status` | JSON String：state、reason、goal_id、pose、plan_count、map_revision、control_permitted |
| `forest_nav_control/task_guard.py` | 监视实际状态、会话、地图契约和定位变换；故障时取消目标并撤销控制许可 |
| `/navigation/control_permitted` | Bool 心跳；桥接端 0.3 秒没有新许可就归零输出 |

状态包括 waiting、idle、accepted、navigating、canceling、succeeded、canceled、failed。
失败原因保存在 reason；旧目标故障后不自动续跑，重新提交目标才能开始新任务。
`plan_count` 是观察到的全局路径发布次数，不代表每一次规划尝试。
这里使用权威统一代价图，更新地图需发布完整 OccupancyGrid。无路可走时终止任务，
不会自动清除地图里的真实障碍或未知区域，也没有加入尚未在 G1 验证的倒退恢复动作。

自动验收：`./test_tasks.sh`。测试运行真实 Nav2 和运动学末端，使用独立测试地图、
ROS_DOMAIN_ID=178、UDP 15381/15382；结果在 `results/tasks_summary.json`。
它验证导航任务机制，不能替代真实 G1 的转弯、绕障、复杂地形与制动能力验证。
旧的 `test_tracking.sh` 和 `navigation_client.py` 仍保留低层 FollowPath 验收接口。

本次实际验收通过：8 项任务集成测试、5 项原有运动学闭环用例、9 项通信与策略输入测试。
动态绕障检查整个半径 0.45 米圆形占地，保守剩余净空约 0.29 米。
最终配置下 G1 通过一米短程坐标导航，终点误差约 0.16 米，见
`results/g1_native_task_report.json`；汇总和源文件摘要见
`results/navigation_tasks_verification.json`。不将运动学绕障结果当作 G1 绕障能力。

绕障停滞排查发现：短前视距离下，DWB 的原地转向与停止候选可能同分。
现采用已安装的 Nav2 RotationShim 先对齐新路径航向，再交给 DWB；
采用原有最大转速 0.5rad/s、角加速度 0.8rad/s²，并保留旋转碰撞检查。
实现参考：[Nav2 Humble 官方源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_rotation_shim_controller/src/nav2_rotation_shim_controller.cpp)。

# 路径跟踪与 G1 策略闭环（第二、三阶段）

本目录新增独立的路径跟踪、实际状态桥接和一键启动。原来的 `start_hiking_forest.sh`、键盘控制及单独策略播放仍可使用，未启用 `--nav2_control` 时不进入新逻辑。

## 当前验证状态（2026-11-12 重启后复验）

**CUDA已经恢复，真实 Isaac PhysX + G1 ONNX 策略的短程导航闭环已通过。** 本次验证的是受控平地场景中约1米目标跟踪、取消停车和失联保护，不代表完整森林长距离导航、松土动力学或真机验证已完成。

| 验证 | 结果 |
|---|---|
| 原有真实 DWB + 运动学末端的5项测试 | 通过并已复验 |
| 真实 G1 约1米目标跟踪 | 通过；最近一次位置误差约0.113米 |
| 真实 G1 取消后停车检查 | 通过；按实际仿真时间和连续反馈窗口判定 |
| 实际策略链路的输入超时归零 | 通过 |
| 实际仿真反馈中断后桥接失效 | 通过 |
| 通信、策略观察写入、xyzw姿态反馈回归 | 共9项测试通过 |

真实物理结果见 `results/isaac_summary.json`；实际根节点轨迹和速度见 `results/isaac_g1_straight.png` 与对应CSV。运动学结果单独保留在 `results/kinematic_summary.json`，没有用它替代G1物理结果。

修复了首次实际接入暴露的姿态约定错误：IsaacLab 3 后端使用xyzw，原先按旧版wxyz读取会误报航向。已改正并补充零度、90度的后端姿态和速度投影测试。

停车检查使用完全位于取消之后的约1秒仿真时间窗口，要求适配器输出为零、净位移小于0.05米、平均平面速度小于0.05m/s、平均转向速度小于0.08rad/s，并设有等待上限。该判据允许人形躯干摆动，不表示每一帧机体速度绝对为零，也不是实机制动性能标定。

反馈中断通过暂停本次独立仿真进程组制造，避免只通知外层shell而实际仿真仍运行；验证后恢复进程，再正常清理。

历史CUDA故障由重启恢复：GPU Recovery Action从Reboot变为None、cuInit返回CUDA_SUCCESS、Isaac内置PyTorch GPU计算通过。驱动仍为595.99.02，未更换驱动或修改内核模块。旧故障证据保留在 `CUDA错误999诊断.md` 和历史日志中。

完整一键启动已另外以 `--headless --exit-on-goal` 实际运行通过，退出码0，见 `results/oneclick_summary.json`；默认显示窗口的模式未在本轮单独验收。

实际策略速度有超调：本轮记录的前向速度峰值约0.52m/s，高于0.35m/s的指令上限。指令限幅不等于实际机体速度被硬性限制，后续仍需标定动态跟踪与制动性能。

## 一键启动真实 G1 导航

当前可直接执行：

```bash
bash /home/ubuntu/robot_projects/forest_nav/nav2_path_tracking/start_navigation.sh \
  --goal -7 -2 0
```

这个命令会依次启动：

1. 独立 CUDA 可用性检查，失败时立即退出，不留下等待黑屏的导航进程。
2. 原始代价地图、状态桥接节点、planner_server、controller_server、生命周期管理和速度适配器。
3. 原有 G1 ONNX 策略；加上可选 `--nav2_control` 入口。
4. 等待实际状态、仿真时钟、TF及服务激活后，用实时起点规划并执行目标。

默认显示 Isaac 窗口；加 `--headless` 无窗口运行。到达目标后策略继续接收零速度以维持站立，Ctrl+C结束本次子进程；加 `--exit-on-goal` 则到达后退出。

当前地图起点 `(-8,-2)` 对应仿真世界 `(0,0)`，默认目标 `(-7,-2)` 为前方约1米。`--goal` 是地图坐标系中的 x y yaw，单位米、米、弧度。

一键启动由 `start_navigation.py` 管理本次创建的进程，不会停止用户另外启动的仿真。若默认本机端口已占用，应停止冲突实例或为整套进程统一设置端口变量。

## 不依赖 G1 的真实 DWB 闭环验收

```bash
bash /home/ubuntu/robot_projects/forest_nav/nav2_path_tracking/test_tracking.sh
```

这仍然运行真实 Nav2 / DWB。只有最终机器人替换为加速度受限的平面运动模型，用来隔离控制接口、时钟、转向、取消和失联问题。测试使用 ROS_DOMAIN_ID=176 和专用本机端口，不驱动默认域的机器人。

真实 G1 短程验收命令：

```bash
bash /home/ubuntu/robot_projects/forest_nav/nav2_path_tracking/test_tracking.sh --backend isaac
```

它会自动启动策略，执行短距离目标、取消和失联测试。本次已实际通过，结果在 `results/isaac_summary.json`。

## 分开启动，便于学习接口

终端一：

```bash
bash /home/ubuntu/robot_projects/forest_nav/nav2_path_tracking/start_tracking.sh
```

终端二：

```bash
bash /home/ubuntu/robot_projects/forest_nav/nav2_path_tracking/start_g1_policy.sh --visualizer kit
```

终端三：

```bash
bash /home/ubuntu/robot_projects/forest_nav/nav2_path_tracking/navigate.sh --goal -7 -2 0
```

三个终端必须使用相同 ROS_DOMAIN_ID 和相同端口设置。外部数据源/机器人未来可直接提供 `/odom`、TF及 `/clock`，不需要把本机 UDP 传输搬到真机。

## 数据通路

```text
代价地图 → planner_server / SmacPlanner2D → nav_msgs/Path
                                           ├─ /plan → g1_state_bridge → Isaac绿色显示
                                           ↓ FollowPath Action
实际 TF、/odom → controller_server / DWB → /cmd_vel [Twist]
                                           ↓ 现有 nav2_velocity_adapter
                                    /cmd_vel_adapted [Twist]
                                           ↓ g1_state_bridge
                                  本机UDP（速度与实际反馈）
                                           ↓
                       G1 command_manager + actor velocity_commands
                                           ↓
                             原有 ONNX 策略 → 关节动作 → 仿真
                                           ↓ 实际根节点位姿与速度
                         g1_state_bridge → /odom、TF、/clock
```

- `/compute_path_to_pose`：调用真实全局规划，使用当前机器人定位，`use_start=false`。
- `/follow_path`：提交路径、`controller_id=FollowPath`、`goal_checker_id=goal_checker`。
- `/navigation/path`：本次提交给控制器的路径，供观察。
- `/plan`：BT当前实际全局路线；桥接节点将其发送到Isaac，绘成无碰撞属性的绿色显示曲线，导航结束后清除。
- `/navigation/bridge_status`：ready、状态新鲜度、会话、位置和故障。
- `/navigation/exploration_status`：当前任务阶段、候选分数、路径检查、子目标失效分类和终点拒绝 cost；不参与控制。
- `/cmd_vel`：DWB输出，机体前向x、左向y、逆时针wz。
- `/cmd_vel_adapted`：已有适配器校验/限幅/限加速度后的指令。
- `/odom`：实际反馈，不能拿 `/cmd_vel` 冒充实际速度。

坐标链为 `map → odom → base_link`。本次受控仿真地图中 map→odom 为单位变换；odom→base_link 来自实际根节点位置与水平航向。base_link 是用于平面导航的水平机体参考系，里程计速度由世界实际速度投影到这个航向，避免人形机体俯仰把垂直速度混入导航。

所有 Nav2 节点使用 `/clock` 仿真时钟。UDP新鲜度、适配器超时和状态断开检查使用单调时钟，仿真暂停也能触发失效处理。

## 为什么使用本机桥接

本机 ROS Humble 使用Python3.10，Isaac使用Python3.12。为避免互相污染依赖，仿真进程只导入标准库通信端点；ROS消息、TF和Action都留在独立ROS节点中。

UDP只绑定127.0.0.1，默认状态端口15361、指令端口15362。消息带协议版本、仿真会话标识、递增序号和同机单调时间。端口可通过 `FOREST_NAV_STATE_PORT`、`FOREST_NAV_COMMAND_PORT` 对整套进程统一配置。

这是当前仿真联调的传输实现，未来真机侧可以直接订阅 `/cmd_vel_adapted` 并发布状态估计，不要求沿用UDP。

## 策略入口和场景的具体改动

InstinctLab只修改了现有 `tasks/parkour/scripts/play.py` 的可选入口，初始文件备份在 `.integration_backup/play_before_nav2.py`。已有用户修改未回退。

`--nav2_control` 启用后：

- 速度来源改为外部指令，取消随机目标速度生成与随机出生；与键盘控制互斥。
- 更新 `command_manager.get_term("base_velocity").vel_command_b`，并按现有键盘控制相同布局覆盖actor的速度观察历史，避免只改ROS缓存却未改策略输入。
- 初始两秒保持零速度，状态健康后才公布ready；跌倒/环境重置触发失败，不悄悄跳回出生点继续旧路径。
- 读取实际 root_pos_w、root_quat_w、root_lin_vel_w、root_ang_vel_w；绝不将指令积分冒充G1真实反馈。
- 使用可重复的平地验证场景，仍沿用原有策略，不训练模型。
- 同一NPZ地图中的100格构建真实碰撞障碍，并加入策略深度传感器目标。高代价区暂为平地上的抽象代价，不能据此声称模拟了松土承载力。

这一受控场景区别于旧脚本生成的随机森林，目的是让地图与物理障碍位置一致。恢复真实复杂地形时，需要重新核对地形、地图和策略能力。

## 当前控制参数及停止行为

| 参数 | 本次测试设置 |
|---|---|
| 前向速度 | 0～0.7 m/s |
| 横向速度 | 固定0；现有训练配置未开放横移 |
| 转向速度 | -0.5～0.5 rad/s |
| 线加速度/减速度 | 0.8 m/s² |
| 角加速度/减速度 | 0.8 rad/s² |
| DWB频率 | 20Hz |
| 适配器发布频率 | 50Hz |
| 目标容差 | 0.25m、0.50rad；按现有G1低速转向能力标定 |
| 占地 | 规划圆半径0.45m，仍为测试值，非实测G1尺寸 |

全局和局部代价地图都保留地形代价、禁止未知，并对障碍和未知边缘膨胀。局部窗口为6×6米，跟随机器人在odom中滚动。

转弯测试曾发现局部地形软代价过重，导致机器人在低代价边界原地等待。已根据真实DWB候选评分将 `BaseObstacle.scale` 调整到0.02。硬性碰撞/未知剔除仍由 critic 非法轨迹判定负责，未通过降低硬性禁行标准解决停滞。

现有策略实测在 `0.096m/s` 纯前进时不持续起步、`0.12m/s` 可走；`0.25rad/s` 原地转向只产生短暂动作，需要约 `0.5rad/s` 才能持续。导航适配器因此只把严格为正且低于阈值的指令抬到 `0.12m/s`，零与纯转向不变；目标判定使用 `0.25m/0.50rad`，避免在策略无法稳定跟踪的末段反复调整。

取消路径由FollowPath Action完成，controller发布零速度，适配器按限制处理；上游超时0.3秒后适配器直接归零。ROS桥接对适配器输出还有自己的新鲜度检查；仿真端独立拒绝超过0.35秒、乱序或不同会话的指令。状态停止超过0.5秒后桥接ready=false。

这些是软件失效保护，不等同于硬件急停。现有运动学末端取消后停车已验证；真实G1仿真已通过上述短程取消停车判据；复杂地形下的制动距离与真机站立稳定性仍需单独标定。

导航模式不使用训练配置里的固定世界高度 `root_height` 终止项：该阈值会在正常下坡时把世界 Z 低于0.5米误判为跌倒。运行时由左右脚高度扫描器直接命中实际地面，检查根节点相对实测地面的净空和机身倾角；扫描不到地面时撤销ready。离线完整高程图只用于路线显示和独立审计。诊断与复测见 `forest_lidar_perception/终止重置诊断_20261113.md`。

## 文件与测试结果

| 文件 | 职责 |
|---|---|
| `config/navigation.yaml` | 全局规划、局部代价地图、DWB和目标/进度检查参数 |
| `launch/tracking.launch.py` | ROS服务和现有速度适配器 |
| `forest_nav_control/bridge_node.py` | 独立ROS桥接节点 |
| `forest_nav_control/sim_endpoint.py` | 无ROS依赖的同机通信与看门狗 |
| `forest_nav_control/isaac_runtime.py` | 可选策略入口、真实状态和一致场景 |
| `forest_nav_control/navigation_client.py` | 实时起点规划、FollowPath、取消和轨迹记录 |
| `forest_nav_control/kinematic_sim.py` | 仅用于接口验收的运动学末端 |
| `forest_nav_control/test_tracking.py` | 自动闭环验收 |
| `forest_nav_control/start_navigation.py` | GPU预检及整套一键启动 |
| `results/kinematic_summary.json` | 当前已通过的软件闭环报告 |
| `results/*_trajectory.csv` | 实际反馈的时间、位置、朝向和速度 |
| `results/path_overlay_test.json` | 实际`/plan`叠加、截图和长目标验收 |

通信测试在ROS的Python环境中运行；策略观察写入测试使用Isaac自带PyTorch的CPU张量，不需要CUDA初始化。已完成Python3.12语法检查和启动脚本语法检查。仓库要求的 `pre_commit run --all-files` 已尝试，但当前未安装pre_commit，因此该项没有执行成功。

需要排查DWB候选轨迹时可设置 `FOREST_NAV_DEBUG_DWB=1`，客户端会在报告旁保存详细评分；默认关闭以减少传输负担。

当前基础场景短程、取消停稳、`(6,2)`长目标和行走中路线叠加均已通过。后续是把相同闭环扩展到更多场景、在线建图、实际运动包络和真机安全控制。

在线探索会把候选目标在原始图和膨胀图中的状态记录到任务事件。候选在运动中被新观测判为占用时记录为 `subgoal_invalidated`，单独计数，不占普通导航失败额度；连续没有候选时由 `FOREST_NAV_NO_CANDIDATE_TIMEOUT` 限制等待时间。该审计字段默认不改变规划或控制指令。

## 2026-11-12：统一任务反馈接口

坐标面板和TaskClient继续调用原生`/navigate_to_pose`，新增`/navigation/task_records`（`std_msgs/String`，JSON，transient-local、可靠传输）。`/navigation/task_status`保留原有安全监视字段，新增`client_records`按客户端实例保存任务记录；这些显示字段不参与控制许可判定。

JSON包含`schema_version=1`、`source`、`current_task`、`active_tasks`、`recent_tasks`。每项记录包含`task_id`、`goal_id`（Nav2 UUID）、`goal={frame_id,x,y,yaw_rad}`、`state`、`reason`、`pose`、`distance_remaining_m`、`action_status`、`error_code`、`error_msg`、`parking_status`及`replaced_by`。目标位置单位米、朝向弧度；GUI输入仍为度。pose是最近反馈快照，实时位姿仍读`/odom`与监视状态。

任务状态：submitting提交中、rejected拒绝、accepted接纳、navigating执行、canceling请求取消、succeeded原生Action成功、canceled原生Action确认取消、failed失败。旧任务迟到结果写入其历史，不覆盖新任务；新目标被拒绝时旧任务仍保留执行状态。请求取消不等于已取消。

**parking_status当前始终为unverified**，界面明确显示“停稳未验证”；本轮没有实现停车验证器，也没有修复既有漂移/朝向问题。Nav2结果消息如含error_code/error_msg则原样记录；Humble消息没有该字段时保留null/空串，不编造具体失败原因。直接绕过这两个项目客户端调用Nav2的外部目标仍只有原有Action监视，不生成完整项目任务记录。

相关实现：`forest_nav_control/task_records.py`（数据与历史）、`task_reporter.py`（ROS发布）、`coordinate_controller.py`（面板）、`task_client.py`（CLI）、`task_guard.py`（汇总）。

接口验收：`tests/test_task_records.py`共11项通过，含真实ROS Action反馈/结果往返及话题接收（服务端为测试替身，无物理运动）；原目标准入7项回归通过。未重新运行Isaac物理仿真或真实面板窗口验收。
