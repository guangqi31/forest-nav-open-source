"""上游FAST-LIO独立启动：只接原始传感器，输出话题及TF全部隔离。"""
import os
import yaml
from pathlib import Path
from datetime import datetime
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from forest_nav_control.sensor_contract import extrinsics, rotation


def generate_launch_description():
    root=Path(__file__).resolve().parent
    output=Path(os.environ.get('FOREST_LIO_OUTPUT',str(root/'results'/('lio_'+datetime.now().strftime('%Y%m%d_%H%M%S')))))
    output.mkdir(parents=True,exist_ok=True)
    t,q=extrinsics()
    estimator=Node(package='fast_lio',executable='fastlio_mapping',name='forest_fast_lio',namespace='lio',
        parameters=[str(root/'config/fast_lio.yaml'),{'map_file_path':str(output/'map.pcd'),
            'mapping.extrinsic_T':t.tolist(),'mapping.extrinsic_R':rotation(q).reshape(-1).tolist()}],
        # 上游使用绝对话题及固定frame名称；内部TF单独隔离，公开输出由适配器统一改名。
        remappings=[('/Odometry','/lio/raw/odometry'),('/path','/lio/raw/path'),
            ('/cloud_registered','/lio/raw/cloud'),('/Laser_map','/lio/raw/map'),
            ('/cloud_registered_body','/lio/raw/body'),('/cloud_effected','/lio/raw/effected'),
            ('/tf','/lio/internal_tf'),('/tf_static','/lio/internal_tf_static')],output='screen')
    adapter=ExecuteProcess(cmd=['/usr/bin/python3','-m','forest_nav_control.lio_adapter'],output='screen')
    # 复用已经构建的CMU地形算法，单独命名空间，不能误订阅原真值定位链路。
    terrain_params=yaml.safe_load((root/'config/terrain.yaml').read_text())['terrainAnalysis']['ros__parameters']
    terrain=Node(package='terrain_analysis',executable='terrainAnalysis',namespace='lio',name='terrainAnalysis',
        parameters=[terrain_params],remappings=[('/registered_scan','/lio/cloud_registered'),
            ('/state_estimation','/lio/lidar_odometry'),('/terrain_map','/lio/terrain_raw'),
            ('/map_clearing','/lio/map_clearing'),('/joy','/lio/joy')],output='screen')
    bridge=ExecuteProcess(cmd=['/usr/bin/python3','-m','forest_nav_control.lio_terrain_bridge'],output='screen')
    mapper=ExecuteProcess(cmd=['/usr/bin/python3','-m','forest_nav_control.lio_costmap'],
        additional_env={'FOREST_LIO_OUTPUT':str(output)},output='screen')
    return LaunchDescription([estimator,adapter,terrain,bridge,mapper])
