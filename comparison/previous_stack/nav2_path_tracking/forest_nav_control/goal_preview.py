"""候选目标的只读预览：复用终点准入，不发送 Action，不生成运动指令。"""
import math
import os
from pathlib import Path


def preview_file():
    """按本次导航的端口隔离显示文件；它只供 Isaac 绘图使用。"""
    port = int(os.environ.get('FOREST_NAV_COMMAND_PORT', '15362'))
    return Path(f'/tmp/forest_nav_goal_preview_{port}.json')


def evaluate_preview(validator, values):
    """允许输入负号、小数点等中间状态；无效时隐藏标记，绝不冒用原点。

    绿色仅代表通过现有终点栅格准入，不代表整条路径可达或真实停车已验证。
    未知/范围外用黄色保留候选意图，不能把尚未测量的地面当成可停车。
    """
    from .goal_validation import GoalRejected
    try:
        x, y, degrees = map(float, values)
        if not all(math.isfinite(v) for v in (x, y, degrees)):
            raise ValueError('非有限坐标')
    except (ValueError, TypeError):
        return dict(visible=False, status='invalid', reason='请填完整的 X、Y、朝向数字。')
    result = dict(visible=True, x=x, y=y, yaw=math.radians(degrees), cost=None)
    if not validator.enabled:
        return dict(result, status='unknown', reason='当前未启用地形检查，停车条件未确认。')
    try:
        check = validator.check((x, y), allow_unobserved=True)
        if check['unknown']:
            reason = '目标在当前地图外，需探索后确认停车条件。' if check['outside'] else '目标尚未观测，停车条件未知。'
            return dict(result, status='unknown', reason=reason)
        result['cost'] = check['cost']
        return dict(result, status='allowed', reason=f"代价 {check['cost']}：符合当前终点检查；停稳仍需执行验证。")
    except GoalRejected as exc:
        result['cost'] = getattr(exc, 'cost', None)
        return dict(result, status='unknown' if exc.reason == 'map_not_ready' else 'blocked', reason=str(exc))
