"""针对真实接入入口检查：提前交接不能取消旧目标；最终模式必须保留。"""
from pathlib import Path
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock
import importlib.util
import math
import yaml
import pytest
from forest_nav_control.tracking_modes import navigation_overrides
from forest_nav_control.unknown_navigation import UnknownNavigation, MissionStopped

ROOT=Path(__file__).resolve().parents[1]


def test_transit_does_not_modify_final_controller_or_collision_checks():
    import copy
    config=yaml.safe_load((ROOT.parent/'forest_lidar_perception/config/navigation.yaml').read_text())
    original=copy.deepcopy(config)
    c,b=navigation_overrides(config, 'dwb')
    assert config==original
    source=config['controller_server']['ros__parameters']['FollowPath'];transit=c['TransitPath']
    assert not transit['rotate_to_goal_heading'] and 'RotateToGoal' not in transit['critics']
    for key in ['max_vel_x','acc_lim_x','BaseObstacle.scale','ObstacleFootprint.scale']:
        assert transit[key]==source[key]
    assert {'BaseObstacle','ObstacleFootprint'}<=set(transit['critics'])
    assert c['transit_goal_checker']['yaw_goal_tolerance']>math.pi
    assert config['controller_server']['ros__parameters']['goal_checker']['yaw_goal_tolerance']==.03
    assert 'nav2_controller_selector_bt_node' in b['plugin_lib_names']


def manager():
    node=object.__new__(UnknownNavigation)
    result=Future();accepted=SimpleNamespace(accepted=True,goal_id=SimpleNamespace(uuid=bytes(16)),get_result_async=lambda:result)
    sent=Future();sent.set_result(accepted)
    node.plan_revision=0;node.latest_plan_goal=None;node.inflated=None
    def send(request):
        node.plan_revision+=1;node.latest_plan_goal=(2.,0.,math.pi)
        return sent
    node.nav=SimpleNamespace(send_goal_async=Mock(side_effect=send))
    node.check=Mock();node.target_state=Mock(return_value={'valid':True,'reason':''});node.cancel_late=Mock();node.await_future=lambda f,*_:f.result()
    node.controller_selector=SimpleNamespace(publish=Mock());node.goal_checker_selector=SimpleNamespace(publish=Mock())
    from geometry_msgs.msg import PoseStamped
    node.pose_message=lambda x:PoseStamped();node.transition=Mock();node.cancel_native=Mock()
    import time
    node.started=time.monotonic()
    node.use_terminal=False;node.pose=[1.,0.,-1.];node.state={'target':[4.,0.,0.]}
    return node,result,accepted


def test_intermediate_returns_while_native_goal_keeps_running():
    node,result,accepted=manager()
    node.check.side_effect=lambda *_:setattr(node,'pose',[1.6,0.,-1.]) if node.nav.send_goal_async.called else None
    assert node.drive((2.,0.,math.pi),False,Mock(),1)
    assert not result.done() and node.native==(accepted,result)
    node.cancel_native.assert_not_called()
    assert node.controller_selector.publish.call_args.args[0].data=='TransitPath'
    assert node.goal_checker_selector.publish.call_args.args[0].data=='transit_goal_checker'


def test_final_does_not_use_intermediate_early_return():
    node,result,accepted=manager();count=0
    def stop(*_):
        nonlocal count
        count+=1
        if count>=4:raise MissionStopped('test_stop')
    node.check=stop
    with pytest.raises(MissionStopped):node.drive((2.,0.,math.pi),True,Mock(),1)
    assert node.controller_selector.publish.call_args.args[0].data=='FollowPath'
    assert node.goal_checker_selector.publish.call_args.args[0].data=='goal_checker'


def test_launch_builds_nodes_from_selected_params_file():
    from launch import LaunchContext
    from launch.substitutions import TextSubstitution
    spec=importlib.util.spec_from_file_location('forest_tracking_launch',ROOT/'launch/tracking.launch.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    nodes=mod.tracking_nodes(LaunchContext(),TextSubstitution(text=str(ROOT.parent/'forest_lidar_perception/config/navigation.yaml')),ROOT)
    assert len(nodes)==2


def test_mppi_uses_native_plugin_and_keeps_map_and_final_checks():
    import copy
    config=yaml.safe_load((ROOT.parent/'forest_lidar_perception/config/navigation.yaml').read_text())
    before=copy.deepcopy(config)
    controller, navigator=navigation_overrides(config)
    assert config==before
    final=controller['FollowPath'];transit=controller['TransitPath']
    assert final['plugin']=='nav2_mppi_controller::MPPIController'
    assert 'primary_controller' not in final
    assert final['vx_max']==1.8 and final['wz_max']==.5
    assert 'GoalAngleCritic' in final['critics']
    assert 'GoalAngleCritic' not in transit['critics']
    assert transit['CostCritic']==final['CostCritic']
    assert final['CostCritic']['collision_cost']==1000000.
    assert final['CostCritic']['consider_footprint'] is True
    assert final['model_dt']==1/config['controller_server']['ros__parameters']['controller_frequency']
    assert 'nav2_is_path_valid_condition_bt_node' in navigator['plugin_lib_names']
    assert 'nav2_globally_updated_goal_condition_bt_node' in navigator['plugin_lib_names']
    assert 'nav2_recovery_node_bt_node' in navigator['plugin_lib_names']


def test_occupied_observation_refresh_keeps_native_action_running():
    node,result,accepted=manager()
    node.target_state.return_value={'valid':False,'reason':'occupied_inflated','inflated_cost':99}
    assert node.drive((2.,0.,math.pi),False,Mock(),1) is False
    assert node.last_drive_failure=='target_invalidated'
    assert node.native==(accepted,result) and not result.done()
    node.cancel_native.assert_not_called()
    assert node.transition.call_args.args[1]=='subgoal_refresh_required'


def test_soft_cost_change_does_not_add_an_active_goal_gate():
    node,result,accepted=manager();count=0
    node.target_state.return_value={'valid':False,'reason':'stopping_cost','cost':26}
    def stop(*_):
        nonlocal count
        count+=1
        if count>=4:raise MissionStopped('test_stop')
    node.check=stop
    with pytest.raises(MissionStopped,match='test_stop'):
        node.drive((2.,0.,math.pi),False,Mock(),1)
    assert node.last_drive_failure==''
    node.cancel_native.assert_not_called()
