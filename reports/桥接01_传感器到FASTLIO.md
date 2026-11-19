# 桥接①阶段记录：Isaac 原生传感器到 FAST-LIO

代码实现完成，ROS 侧检查通过；Isaac 实际传感器端到端验收尚未通过，未连接旧仿真。

## 本次实际文件操作

- 新增 src/forest_sensor_bridge 下 7 个实现/配置/包声明文件及 README.md。
- 修改 sources.lock.json：登记 FAST-LIO、ikd-Tree 固定提交及本阶段检查状态。
- 新增本报告。
- 原 sources.lock.json 已备份到 backups/bridge01_before/sources.lock.json。
- 本轮新文件 API/构建修正前快照位于 /tmp/forest_sensor_graph_before_api_review.py、
  /tmp/forest_bridge01_package_before_build_fix.xml。
- 从本机复用 FAST-LIO 和 ikd-Tree，没有网络下载，没有安装软件依赖。
- 生成 build、install、log 构建目录；未把检查脚本加入生产链路。

## 自写与开源边界

自写：传感器装配、点云字段转换、launch 配置组合、安装位姿到外参的坐标换算。
配置：雷达10Hz、IMU目标200Hz、话题、坐标系、安装位姿及 FAST-LIO 原有参数。
开源原样：NVIDIA 传感器与发布节点、FAST-LIO、ikd-Tree、CMU、FAR、Go2参考工程。
全部相关上游 Git 工作区核对为干净。本次没有编写定位、建图、规划或速度控制算法。
没有新增任务限时、速度限制、强制对齐、任务否决、失败退出或自动重置逻辑。
数据格式错误只报告当前点云；不会终止其他进程或生成伪造数据。

## 已执行检查

1. Python 语法、ROS包清单、YAML读取通过。
2. 临时构造消息核对：大小端、二维行填充、64位时间、点顺序、帧头、frame、字段、ring、空帧通过。
3. 独立 ROS 域181下，实际输入发布→适配节点→输出订阅通过；错误消息后合法消息仍能通过。
4. fast_lio 和 forest_sensor_bridge 构建通过；launch参数解析通过。
5. 独立域短暂启动 launch，FAST-LIO 显示 Node init finished，输入话题、秒单位及rad/s参数正确。
   没有输入真实传感器数据，不能据此证明定位/建图成功。检查后只关闭自己启动的进程。
6. 上游工作区均干净。旧项目未写入、未重启、未关闭。

检查中修复：本机克隆最初未带 ikd-Tree，随后从本机补齐；新 package.xml
占位邮箱格式不符合 ROS 清单规范，已修正。首次DDS检查受沙箱权限和测试域编号影响，
更换合法独立域、使用本机套接字权限后通过。首次启动检查停止时，对进程组及launch
重复发送SIGINT产生清理日志；不属于运行中故障，未为此添加生产退出逻辑。

证据：/tmp/check_forest_bridge01.py、/tmp/forest_bridge01_launch.log、
项目log目录下的构建记录。临时脚本未安装，/tmp内容可能由系统清理。

## 未执行 / 剩余缺口

- 工具进程 nvidia-smi 无法与驱动通信，未启动 RTX 雷达或真实 G1 场景验证。
- sensor_graph.py 尚未挂接宿主 G1 仿真入口；本轮没有新增或修改宿主入口。
- 雷达资产实际可访问性、32线通道定义、逐点时间跨度和运动时坐标语义未实测。
- IMU静止比力、角速度轴向、采样频率及与雷达的时间同步未实测。
- FAST-LIO的真实定位/建图未验收；后续桥接②/③均未开始。

调用方式见 src/forest_sensor_bridge/README.md。下一步应只进行本桥接的实际场景核对，
不应把它直接表述为全链路导航完成。
