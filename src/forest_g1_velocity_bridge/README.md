# 桥接③：速度指令 → G1策略

只做接口适配，没有规划、控制或保护算法。没有限速、平滑、超时归零、
帧/时间拒绝、速度仲裁、摔倒判断、自动重置或退出逻辑。
收到新消息前保持现有缓存，首次消息前为[0,0,0]。

## 输入输出

输入：/cmd_vel，geometry_msgs/msg/TwistStamped。
输出：[twist.linear.x, twist.linear.y, twist.angular.z]，单位m/s、m/s、rad/s。
上游CMU的vehicle与G1策略基座必须采用相同坐标轴（x前、y左、z上）；这里不做TF计算。
CMU当前只填写vx、wz，vy默认为0，接口保留完整的三列。

原生ROS2Subscriber直接接收TwistStamped；不增加ROS中转节点，不混用Python版本。
替换policy/critic里的velocity_commands取值函数，沿用原生ObservationManager维护8帧历史。
不改历史长度、观测顺序、策略权重、动作输出、速度命令算法或开源推理脚本。

## 宿主调用

Isaac进程先按桥接①在SimulationApp初始化前加载大点云传输配置。
然后让Python能导入本包。源码目录为：
/home/ubuntu/robot_projects/forest_nav_open_source/src/forest_g1_velocity_bridge
安装后目录为：
/home/ubuntu/robot_projects/forest_nav_open_source/install/forest_g1_velocity_bridge/share/forest_g1_velocity_bridge/python

```python
from forest_g1_velocity_bridge import G1VelocityAdapter

# 环境配置准备好后、创建环境前：
velocity_adapter = G1VelocityAdapter(topic="/cmd_vel")
velocity_adapter.configure_observations(env_cfg)

# 宿主照常创建环境、载入原来的策略；环境创建后、首次推理前：
velocity_adapter.connect()
obs, _ = env.get_observations()
# 后续照常 policy(obs) 与 env.step(actions)。
```

使用此输入时，宿主不再启用旧nav2_control桥接或键盘覆盖；本包不提供输入仲裁。
如果宿主是旧play.py的键盘/nav2专用分支，不能将其已有obs覆盖继续叠加到此接口。

## 独立G1仿真入口

运行 `/home/ubuntu/robot_projects/forest_nav_open_source/start_g1_ros.sh`。
本入口启动G1环境、加载现有ONNX权重并把本包接入原生观测更新。
复用本机旧play.py的配置解析和ONNX模型加载函数，但不执行它的旧导航运行循环。
旧项目文件不修改；不启动旧导航、FAR或桥接②，也不自动加载旧森林扩展。

新增宿主为 scripts/play_g1_ros.py。终止项在宿主配置中清空，避免训练回合时限或摔倒重置。
循环持续执行ONNX策略和env.step，不根据任务状态自行退出。
G1_CMD_VEL_TOPIC可指定速度话题，默认/cmd_vel；ROS_DOMAIN_ID遵循环境设置。
当前只支持本机已有的G1 parkour ONNX模型及其现有观测结构。


## 复杂林地实际联合测试更新

默认G1入口已加载assets/forest_complex中的既有复杂林地资产，不调用旧导航逻辑。
雷达真实采样原点为torso_link上方0.65m，IMU仍在躯干原点；原0.25m位置落在胸部模型内，不能用于G1。
安装配置由sensors.yaml共享给仿真与FAST-LIO，extrinsic_T由原生启动配置计算。
IsaacLab宿主使用PhysX原生后步回调，ROS时钟、IMU和RTX外部Fabric时间统一到真实物理时间。
真实雷达/IMU→FAST-LIO→CMU/FAR→G1已跑通，但目标到达后出现越过、回摆并摔倒，复杂导航验收未通过。
没有新增限速、停车保护、失败退出或自动恢复；原生算法参数仍为未标定基线。详见reports/复杂林地全流程测试.md。


## 自写入口性能修复（2026-11-19）

独立ONNX入口只计算policy观测；不再计算critic、AMP及动作参考，训练奖励使用原生零权重跳过分支。policy输入结构、历史和模型保持不变。通用适配器仍可由其他宿主显式选择观测组。

传感器与速度订阅图的运行期属性写入指定undoable=False，不再生成界面编辑历史。物理步长、传感器频率、消息值和QoS不变。

节拍等待按每一步独立计时，落后或暂停后不追赶旧期限；计算本身超过预算时仍可能低于实时，不能视作已保证1:1。

start_g1_sensors.sh用文件锁复用已有实例；用户Ctrl+C时清理本次创建的G1与ROS进程组。没有按失败、摔倒或任务超时自动关闭其他进程。若G1自身异常结束而ROS还在运行，先在原入口Ctrl+C，再重新启动；不自动重启或隐藏异常。
