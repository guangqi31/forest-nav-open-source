"""启动地图、状态桥、全局规划、局部跟踪、适配器和生命周期管理。"""
from pathlib import Path
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def tracking_nodes(context, params, root):
    """按实际params_file派生途中模式，保留原终点检查器及停车闭环。"""
    import yaml
    # ros2 launch入口不保证项目包位于PYTHONPATH，按本启动文件定位辅助模块。
    from runpy import run_path
    navigation_overrides = run_path(str(root/'forest_nav_control/tracking_modes.py'))['navigation_overrides']
    with open(params.perform(context), encoding='utf-8') as stream:
        backend = os.environ.get('FOREST_NAV_CONTROLLER', 'mppi')
        config = yaml.safe_load(stream)
        controller, navigator = navigation_overrides(config, backend)
    print(f'[ForestNav] 局部控制器: {backend}；地图/策略/终点闭环沿用现有配置')
    navigator.update(default_nav_to_pose_bt_xml=str(root/('config/navigate_mppi.xml' if backend == 'mppi' else 'config/navigate.xml')),
                     default_nav_through_poses_bt_xml=str(root/'config/navigate_through.xml'))
    far = os.environ.get('FOREST_NAV_PLANNER') == 'far'
    if far:
        if backend!='mppi':raise RuntimeError('FAR执行链需要MPPI')
        # 路径滚动更新，不将每段可执行前缀当成最终停车目标。
        for critic in ('GoalCritic','PathFollowCritic','PathAlignCritic','PathAngleCritic'):
            controller['TransitPath'][critic]['threshold_to_consider'] = .25
    controller_params = [params, controller]
    if backend == 'mppi':
        import math
        import json
        import tempfile
        local = config['local_costmap']['local_costmap']['ros__parameters']
        # Nav2 makeFootprintFromRadius本来就把圆生成16个点；显式提供同一几何，
        # 使Humble MPPI使用与旧DWB相同的完整足迹检查。半径、padding均不增加。
        footprint = local.get('footprint')
        if not footprint:
            radius = local['robot_radius']
            footprint = json.dumps([[radius*math.cos(i*math.tau/16),
                                     radius*math.sin(i*math.tau/16)] for i in range(16)])
        with tempfile.NamedTemporaryFile(mode='w', prefix='forest_mppi_footprint_',
                                         suffix='.yaml', delete=False) as temp:
            yaml.safe_dump({'local_costmap': {'local_costmap': {'ros__parameters':
                           {'footprint': footprint}}}}, temp)
            controller_params.append(temp.name)
    terminal = os.environ.get('FOREST_NAV_MAP_MODE') == 'lidar' and os.environ.get('FOREST_NAV_TERMINAL_CONTROL') == '1'
    nodes = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=controller_params,
             output='screen'),
    ]
    if not far:
        nodes.append(Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             parameters=[params, navigator], output='screen'))
    return nodes


def generate_launch_description():
    root=Path(__file__).resolve().parents[1]
    project=root.parent
    params=LaunchConfiguration('params_file')
    mode = os.environ.get("FOREST_NAV_MAP_MODE", "offline")
    online = mode == "online"
    far = os.environ.get('FOREST_NAV_PLANNER') == 'far'
    if far and (mode != 'lidar' or os.environ.get('FOREST_NAV_LOCALIZATION') != 'lio'):
        raise RuntimeError('FAR模式当前必须使用lidar地图与LIO估计定位')
    online_processes = []
    terminal = mode == 'lidar' and os.environ.get('FOREST_NAV_TERMINAL_CONTROL') == '1'
    if online:
        online_processes = [
            ExecuteProcess(cmd=["bash",str(project/"forest_online_mapping/start.sh")],output="screen"),
            ExecuteProcess(cmd=["/usr/bin/python3","-m","forest_nav_control.cloud_gateway"],cwd=str(root),output="screen")]
    if mode == 'lidar':
        online_processes = [
            ExecuteProcess(cmd=['/usr/bin/python3','-m','forest_nav_control.lidar_bridge'],cwd=str(root),output='screen'),
            ExecuteProcess(cmd=['bash',str(project/'forest_lidar_perception/start_cmu.sh')],output='screen'),
            ExecuteProcess(cmd=['/usr/bin/python3','-m','forest_nav_control.terrain_costmap'],cwd=str(root),output='screen'),
            ExecuteProcess(cmd=['bash',str(project/'forest_lidar_perception/start_frontier.sh')],output='screen'),
            ExecuteProcess(cmd=['/usr/bin/python3','-m','forest_nav_control.unknown_navigation'],cwd=str(root),output='screen')]
        if os.environ.get('FOREST_NAV_LOCALIZATION')=='lio':
            # 互斥切换：保留传感器桥、探索及任务接口，撤掉原真值CMU/地图发布者。
            online_processes=[
                ExecuteProcess(cmd=['/usr/bin/python3','-m','forest_nav_control.lidar_bridge'],cwd=str(root),output='screen'),
                ExecuteProcess(cmd=['bash',str(project/'forest_lidar_perception/start_lio.sh')],output='screen'),
                ExecuteProcess(cmd=['/usr/bin/python3','-m','forest_nav_control.lio_navigation'],cwd=str(root),output='screen'),
                ExecuteProcess(cmd=['/usr/bin/python3','-m','forest_nav_control.lio_plan_visualization'],cwd=str(root),output='screen'),
                Node(package='robot_localization',executable='ekf_node',name='forest_lio_ekf',
                    parameters=[str(project/'forest_lidar_perception/config/lio_ekf.yaml')],
                    remappings=[('/odometry/filtered','/odom')],output='screen'),
                ExecuteProcess(cmd=['bash',str(project/'forest_lidar_perception/start_frontier.sh')],output='screen'),
                ExecuteProcess(cmd=['/usr/bin/python3','-m','forest_nav_control.unknown_navigation'],cwd=str(root),output='screen')]
    if far:
        online_processes=online_processes[:-2]
        online_processes += [
            ExecuteProcess(cmd=['bash',str(project/'forest_far_navigation/start_far.sh')],output='screen'),
            ExecuteProcess(cmd=['/usr/bin/python3','-m','forest_nav_control.far_inputs'],cwd=str(root),output='screen'),
            ExecuteProcess(cmd=['/usr/bin/python3','-m','forest_nav_control.far_navigation'],cwd=str(root),output='screen')]
    if terminal:
        online_processes.append(ExecuteProcess(cmd=['/usr/bin/python3','-m','forest_nav_control.terminal_node'],cwd=str(root),output='screen'))
    return LaunchDescription(online_processes + [
        DeclareLaunchArgument('publish_map',default_value='false' if mode in ('online','lidar') else 'true'),
        DeclareLaunchArgument('params_file',default_value=os.environ.get('FOREST_NAV_PARAMS',str(root/'config/navigation.yaml'))),
        DeclareLaunchArgument('map_file',default_value=os.environ.get('FOREST_NAV_MAP',str(project/'forest_costmap_test/forest_costmap.npz'))),
        ExecuteProcess(cmd=['/usr/bin/python3',str(project/'forest_costmap_test/publish_costmap.py'),
            '--ros-args','-p',['map_file:=',LaunchConfiguration('map_file')],'-p','use_sim_time:=true'],condition=IfCondition('false' if mode in ('online','lidar') else LaunchConfiguration('publish_map'))),
        ExecuteProcess(cmd=['/usr/bin/python3','-m','forest_nav_control.bridge_node'],cwd=str(root),output='screen'),
        ExecuteProcess(cmd=['/usr/bin/python3','-m','nav2_velocity_adapter.node',
            '--publish-hz','50',
            '--ros-args','-p','use_sim_time:=true'],cwd=str(project/'nav2_velocity_adapter'),output='screen'),
        *([] if far else [Node(package='nav2_planner',executable='planner_server',name='planner_server',parameters=[params],output='screen')]),
        OpaqueFunction(function=tracking_nodes, args=[params, root]),
        ExecuteProcess(cmd=['/usr/bin/python3','-m','forest_nav_control.task_guard'],cwd=str(root),output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='tracking_lifecycle_manager',
             parameters=[{'use_sim_time':True,'autostart':True,'node_names':['controller_server'] if far else ['planner_server','controller_server','bt_navigator']}],output='screen'),
    ])
