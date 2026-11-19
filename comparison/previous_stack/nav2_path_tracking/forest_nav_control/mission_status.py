"""固定上层任务阶段契约；纯状态解释，不另写规划或控制调度器。"""
PHASE_NAMES = {
    'waiting_ready':'等待系统就绪', 'waiting_observation':'等待观测', 'ready':'等待目标',
    'navigating':'导航中', 'canceling':'正在取消', 'arrived':'Nav2已到达（停稳另验）',
    'canceled':'已取消', 'blocked':'任务受阻', 'fault':'系统故障',
    'settling':'正在收步并核对停稳',
    'holding':'到达后重新纠偏', 'hold_fault':'位置保持失效，已撤销纠偏',
    'exploring':'探索中',  # 预留状态，本阶段不实现或自动进入探索。
}


def interpret_phase(state,active,healthy,mapping_ready,request=None,latched_fault=False):
    """动作与系统健康分开；新请求被拒绝时不能掩盖正在执行的旧任务。"""
    request=request or {}
    if latched_fault:return 'fault'
    if not healthy:
        return 'fault' if active else ('waiting_observation' if not mapping_ready else 'waiting_ready')
    if active:return 'canceling' if state=='canceling' else 'navigating'
    if request.get('state')=='rejected':
        return 'waiting_observation' if request.get('reason_code') in ('unknown','map_not_ready') else 'blocked'
    return {'succeeded':'arrived','canceled':'canceled','failed':'blocked'}.get(state,'ready')


def manager_fault(status, received, now):
    """上层管理器失联或取消未确认时撤销控制许可；恢复数据不会续跑旧任务。

    普通终点受阻不锁死系统，用户可以重新指定目标。取消确认超时则必须重启
    本次导航会话，不能在尚未确认旧控制器停止时再发新任务。
    """
    if now-received>1.:return 'mission_manager_timeout','未知导航管理器状态超时'
    if status.get('reason_code')=='cancel_confirmation_timeout':
        return 'cancel_confirmation_timeout','未确认旧目标已停止，请重启本次导航会话'
    return '',''
