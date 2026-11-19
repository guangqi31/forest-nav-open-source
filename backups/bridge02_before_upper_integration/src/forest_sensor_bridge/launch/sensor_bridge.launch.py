"""启动传感器字段适配与原生 FAST-LIO，不启动仿真、规划或控制。"""
from pathlib import Path
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rotation(wxyz):
    """安装四元数到旋转矩阵，只用于坐标接口，不参与状态估计。"""
    q = np.asarray(wxyz, dtype=float)
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def _nodes(context):
    config_path = LaunchConfiguration('sensors_config').perform(context)
    with open(config_path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    lidar, imu = config['lidar'], config['imu']
    # 在两个子进程创建DDS上下文之前加载同一传输配置。
    # 只增大共享内存容量；保留上游和本包现有的SensorDataQoS。
    transport_env = {'FASTRTPS_DEFAULT_PROFILES_FILE':
                     LaunchConfiguration('transport_profile').perform(context)}
    # 两个安装位姿都相对同一个刚体。FAST-LIO 要的是 p_I = R_IL p_L + t_IL。
    rotation_imu = _rotation(imu['orientation_wxyz'])
    rotation_lidar = _rotation(lidar['orientation_wxyz'])
    rotation_il = rotation_imu.T @ rotation_lidar
    translation_il = rotation_imu.T @ (np.array(lidar['position']) - np.array(imu['position']))
    return [
        Node(package='forest_sensor_bridge', executable='pointcloud_fields.py', output='screen',
             additional_env=transport_env,
             parameters=[{'use_sim_time': True, 'input_topic': lidar['raw_topic'],
                          'output_topic': lidar['output_topic'], 'frame_id': config['frames']['lidar'],
                          'scan_lines': int(lidar['scan_lines'])}]),
        Node(package='fast_lio', executable='fastlio_mapping', output='screen',
             additional_env=transport_env,
             condition=IfCondition(LaunchConfiguration('start_fast_lio')),
             parameters=[LaunchConfiguration('fast_lio_config'),
                         {'use_sim_time': True, 'common.lid_topic': lidar['output_topic'],
                          'common.imu_topic': imu['topic'],
                          'preprocess.scan_line': int(lidar['scan_lines']),
                          'mapping.extrinsic_R': rotation_il.reshape(-1).tolist(),
                          'mapping.extrinsic_T': translation_il.tolist()}]),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('forest_sensor_bridge'))
    return LaunchDescription([
        DeclareLaunchArgument('sensors_config', default_value=str(share / 'config/sensors.yaml')),
        DeclareLaunchArgument('fast_lio_config', default_value=str(share / 'config/fast_lio_sim.yaml')),
        DeclareLaunchArgument('transport_profile', default_value=str(share / 'config/fastdds_large_points.xml')),
        DeclareLaunchArgument('start_fast_lio', default_value='true',
                              description='可单独打开消息适配，供传感器接口核对'),
        OpaqueFunction(function=_nodes),
    ])
