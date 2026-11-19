"""从当前Nav2配置派生途经模式，避免把观察点当作需要停稳的终点。"""
from copy import deepcopy
import math


def navigation_overrides(config, backend='mppi'):
    """派生途中模式；地图、终点检查器与输入配置均不被修改。

    MPPI直接输出连续vx/wz；途中只移除终点朝向评分，不经过RotationShim。
    dwb分支保留旧版本，供显式回退，不在运行失败时自动切换控制器。
    """
    base = config['controller_server']['ros__parameters']
    if backend == 'mppi':
        from pathlib import Path
        import yaml
        final = yaml.safe_load((Path(__file__).resolve().parents[1] /
                                'config/controller_mppi.yaml').read_text())
        transit = deepcopy(final)
        transit['critics'].remove('GoalAngleCritic')
    elif backend == 'dwb':
        final = deepcopy(base['FollowPath'])
        transit = deepcopy(final)
        transit['rotate_to_goal_heading'] = False
        transit['critics'] = [name for name in transit['critics'] if name != 'RotateToGoal']
    else:
        raise ValueError(f'未知导航控制器: {backend}，应为mppi/dwb')
    # 途经点只检查位置；不要求指定yaw，也不要求先停到零速度。
    checker = deepcopy(base['goal_checker'])
    checker['plugin'] = 'nav2_controller::SimpleGoalChecker'
    checker['yaw_goal_tolerance'] = math.tau
    controller = {
        'controller_plugins': list(dict.fromkeys([*base['controller_plugins'], 'TransitPath'])),
        'goal_checker_plugins': list(dict.fromkeys([*base['goal_checker_plugins'], 'transit_goal_checker'])),
        'FollowPath': final,
        'TransitPath': transit,
        'transit_goal_checker': checker,
    }
    plugins = config['bt_navigator']['ros__parameters']['plugin_lib_names']
    navigator = {'plugin_lib_names': list(dict.fromkeys([
        *plugins, 'nav2_controller_selector_bt_node', 'nav2_goal_checker_selector_bt_node',
    ]))}
    if backend == 'mppi':
        navigator['plugin_lib_names'] += [
            'nav2_globally_updated_goal_condition_bt_node',
            'nav2_is_path_valid_condition_bt_node',
            'nav2_recovery_node_bt_node',
        ]
    return controller, navigator
