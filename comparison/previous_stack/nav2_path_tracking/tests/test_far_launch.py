"""启动图检查：FAR只保留Nav2局部控制，不能暗中回到旧探索/Smac。"""
from pathlib import Path
import importlib.util
from launch import LaunchContext
from launch.substitutions import TextSubstitution
from launch.actions import ExecuteProcess
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

ROOT=Path(__file__).resolve().parents[1]


def test_far_launch_removes_old_planning_stack(monkeypatch):
    monkeypatch.setenv('FOREST_NAV_PLANNER','far');monkeypatch.setenv('FOREST_NAV_MAP_MODE','lidar')
    monkeypatch.setenv('FOREST_NAV_LOCALIZATION','lio');monkeypatch.setenv('FOREST_NAV_CONTROLLER','mppi')
    monkeypatch.setenv('FOREST_NAV_TERMINAL_CONTROL','1')
    spec=importlib.util.spec_from_file_location('far_tracking_launch',ROOT/'launch/tracking.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    nodes=module.tracking_nodes(LaunchContext(),TextSubstitution(text=str(ROOT.parent/'forest_lidar_perception/config/navigation.yaml')),ROOT)
    assert len(nodes)==1
    description=module.generate_launch_description()
    context=LaunchContext();context.launch_configurations['map_file']='unused'
    commands=[]
    for action in [*description.entities,*nodes]:
        if isinstance(action,Node):commands.append(str(action.node_package))
        elif isinstance(action,ExecuteProcess):
            commands.append(' '.join(perform_substitutions(context,arg) for arg in action.cmd))
    text='\n'.join(commands)
    assert 'planner_server' not in text
    assert 'bt_navigator' not in text
    assert 'unknown_navigation' not in text
    assert 'start_frontier.sh' not in text
    assert 'far_navigation' in text and 'start_far.sh' in text
