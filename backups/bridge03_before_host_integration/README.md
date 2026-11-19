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

## 宿主调用（本轮没有修改宿主）

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

## 本轮范围

仅提供适配包及接入调用，不修改或启动旧G1脚本，不接桥接②，不运行规划。
首次G1联合运行、具体权重和宿主入口的挂接仍须单独完成，不能把接口检查说成机器人已运行。
