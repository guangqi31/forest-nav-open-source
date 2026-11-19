"""只启动上层原生导航节点，不创建传感器、G1、车辆模拟器或自写算法。"""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    interface = LaunchConfiguration('interface_config').perform(context)
    navigation = LaunchConfiguration('navigation_config').perform(context)
    with open(navigation, encoding='utf-8') as stream:
        params = yaml.safe_load(stream)
    planner_share = Path(get_package_share_directory('local_planner'))
    far_share = Path(get_package_share_directory('far_planner'))
    decoder_share = Path(get_package_share_directory('graph_decoder'))
    with (decoder_share / 'config/default.yaml').open(encoding='utf-8') as stream:
        decoder_params = yaml.safe_load(stream)
    # 原生localPlanner和pathFollower共用安装偏移；当前IMU在躯干原点，基线为0。
    offsets = params['localPlanner']['ros__parameters']
    offset_x, offset_y = float(offsets['sensorOffsetX']), float(offsets['sensorOffsetY'])

    def static_tf(name, parent, child, x=0., y=0., z=0.):
        return Node(package='tf2_ros', executable='static_transform_publisher', name=name,
                    arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                               '--yaw', '0', '--pitch', '0', '--roll', '0',
                               '--frame-id', parent, '--child-frame-id', child],
                    parameters=[{'use_sim_time': True}], output='screen')

    return [
        Node(package='loam_interface', executable='loamInterface', name='loamInterface',
             parameters=[interface], output='screen'),
        # 地图原点使用FAST-LIO自己的初始坐标，不假装与Isaac世界原点重合。
        static_tf('map_lio_alias', 'map', 'camera_init'),
        static_tf('body_sensor_alias', 'body', 'sensor'),
        static_tf('sensor_vehicle_mount', 'sensor', 'vehicle', -offset_x, -offset_y),
        Node(package='terrain_analysis', executable='terrainAnalysis', name='terrainAnalysis',
             parameters=[navigation], output='screen'),
        Node(package='terrain_analysis_ext', executable='terrainAnalysisExt', name='terrainAnalysisExt',
             parameters=[navigation], output='screen'),
        Node(package='far_planner', executable='far_planner', name='far_planner',
             parameters=[str(far_share / 'config/default.yaml'), navigation],
             remappings=[('/odom_world', '/state_estimation'),
                         ('/terrain_cloud', '/terrain_map_ext'),
                         # Scan回调使用扫描点；TerrainLocal回调按intensity地形高度分类。
                         ('/scan_cloud', '/registered_scan'),
                         ('/terrain_local_cloud', '/terrain_map')], output='screen'),
        Node(package='local_planner', executable='localPlanner', name='localPlanner',
             parameters=[navigation, {'pathFolder': str(planner_share / 'paths')}], output='screen'),
        Node(package='local_planner', executable='pathFollower', name='pathFollower',
             parameters=[navigation],
             remappings=[('/cmd_vel', LaunchConfiguration('cmd_vel_topic'))], output='screen'),
        Node(package='graph_decoder', executable='graph_decoder', name='graph_decoder',
             parameters=[decoder_params, {'use_sim_time': True}], output='screen'),
        Node(package='rviz2', executable='rviz2', name='forest_navigation_rviz',
             arguments=['-d', str(far_share / 'rviz/default.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz')),
             output='screen'),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('forest_navigation_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('interface_config', default_value=str(share / 'config/interface.yaml')),
        DeclareLaunchArgument('navigation_config', default_value=str(share / 'config/navigation.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        # 用于接口检查时输出到独立话题；正常接G1保持默认/cmd_vel。
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        OpaqueFunction(function=_nodes),
    ])
