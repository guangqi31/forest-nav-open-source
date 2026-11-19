# 林地导航：开源配套链路重建

独立目录：/home/ubuntu/robot_projects/forest_nav_open_source

现阶段：第1阶段源码基线已完成；未编译、未运行、未验证导航。

- src/cmu_autonomy：官方Humble开发环境，完整拉取，包含local_planner、pathFollower、terrain_analysis、terrain_analysis_ext和官方仿真器。
- src/far_planner：官方humble-jazzy分支，全局路线/航点规划。
- references/autonomy_stack_go2：CMU腿式完整工程foxy-humble分支，参考用途，不参与默认构建。
- sources.lock.json：完整提交、来源、源码树、包清单及实际检查结果。
- docs/原生接口与阶段边界.md：原生连接和自写范围。
- reports/阶段01_源码基线.md：本阶段如实汇报。

第2阶段才检查安装依赖、编译并运行官方示例。环境模型由官方另外下载，本阶段未下载；不能宣称示例已可运行。G1及Isaac的接口适配尚未编写。FAST-LIO接入属于第4阶段，暂未把旧项目的修改版本复制过来。

后续按“官方原版→Isaac/G1→定位→局部导航→FAR→森林→面板”的阶段顺序推进。
