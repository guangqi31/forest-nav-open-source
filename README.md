# Forest navigation — G1 / Isaac Sim 全链路诊断快照

这是当前实际运行工作树的完整源码审查快照，上传日期为2026-11-19。**当前导航和1:1实时验收没有通过。** 不要将历史报告中的阶段成功理解为全系统已经完成。

## 从哪里开始看

先读 [AI排查入口.md](AI排查入口.md)。里面列出了实际架构、自写范围、性能证据和未解决问题。

|目录|内容|
|---|---|
|`src/forest_*`|自写适配器、启动宿主与导航配置|
|`src/cmu_autonomy`、`src/far_planner`、`src/fast_lio`|当前固定版本的开源源码，已展开，包含FAST-LIO的ikd-Tree|
|`assets/forest_complex`|实际使用的三维林地USD及网格|
|`runtime_sources`|本机实际导入的InstinctLab、IsaacLab和instinct_rl工作树；包含本机已有修改，不冒称纯净上游|
|`runtime_models`|本机现有策略、动作参考数据及随附文件|
|`reports`|阶段报告、真实测量及失败证据|
|`backups`|新项目各阶段自写文件备份|
|`comparison/previous_stack`|旧版自写接线与配置，供分析版本差异；不由新入口加载|
|`references`|参考工程，不加入默认colcon构建|

上游提交见`sources.lock.json`；实际依赖工作树来源和修改状态见`SNAPSHOT_MANIFEST.json`。上游原许可证保留，没有对第三方源码重新声明许可证。

## 复现边界

这是审查快照，**不是Docker镜像或可直接在任意机器一键运行的发行包**。Isaac Sim 6.2.0、ROS Humble、驱动与已安装Python包需要另行准备；没有打包Isaac安装目录、系统环境、编译产物或认证信息。

保留生产入口原样，里面仍有原机器绝对路径；未为了上传改运行逻辑。迁移时需要安装runtime_sources中的实际依赖版本，并核对`G1_PLAY_SOURCE`、`HIKING_MODEL_ROOT`、`HIKING_DATA_ROOT`和Isaac路径。仅设置G1_PLAY_SOURCE不等于全部Python包都已指向正确工作树。

原机启动G1及传感器：`./start_g1_sensors.sh`。在另一个已source ROS与本工作区install环境的终端启动：`ros2 launch forest_navigation_bringup navigation.launch.py rviz:=false`。新机器先构建src下的运行包；references与runtime_sources不是此colcon命令的源码根目录。

未上传的媒体、存档及外部链接见`OMITTED_FILES.json`；构建、安装、日志缓存目录和Git内部历史统一排除。源码、现有报告、模型及森林资产已随快照保留。旧版结果大目录没有整份复制，相关对比证据已另存。

历史原README放在`docs/历史README_原样.md`，其中“未编译、未运行”等早期描述已经过时。

打包中的Git LFS边界及原始占位资源见[environment/快照打包说明.md](environment/快照打包说明.md)。

原本就未下载的13个上游测试/文档LFS资源不在本仓库，原占位信息已归档；当前森林、模型和运行源码不受影响。
