"""终点闭环的行为验收：误差收敛、保持再纠偏、速度互斥和碰撞拒绝。

小模型只验证外环控制逻辑，不代替实际 G1 策略与足底接触测试。
"""
import math
from types import SimpleNamespace as NS
import pytest
from forest_nav_control.terminal_core import TerminalConfig, TerminalController, trajectory_clear


def test_pose_convergence_then_drift_reacquires_same_target():
    c=TerminalController((.55,.12,.2));pose=[0.,0.,0.];velocity=(0.,0.,0.)
    for _ in range(5000):
        command=c.step(pose,velocity,{'settled':not any(velocity)},.05)
        vx,_,wz=command
        assert 0<=vx<=c.cfg.max_forward_mps and command[1]==0
        pose[0]+=vx*math.cos(pose[2])*.05;pose[1]+=vx*math.sin(pose[2])*.05;pose[2]+=wz*.05
        velocity=command
        if c.verified:break
    assert c.verified and math.dist(pose[:2],c.target[:2])<.25
    assert abs(pose[2]-.2)<.03
    original=c.target;pose[0]-=.3
    assert any(c.step(pose,(0,0,0),{'settled':True},.05))
    assert not c.verified and c.phase=='correcting' and c.target==original


def test_target_behind_turns_without_reverse_or_sideways():
    c=TerminalController((-.5,0,0))
    vx,vy,wz=c.step((0,0,0),(0,0,0),{'settled':True},.02)
    assert vx==vy==0 and abs(wz)>0


def test_small_position_overshoot_does_not_turn_back_during_final_alignment():
    c=TerminalController((1.,0.,0.),TerminalConfig(reacquire_fraction=1.))
    first=c.step((.9,0.,.1),(0,0,0),{'settled':False},.05)
    assert c.position_acquired and first[0]==0 and first[2]<0
    after=c.step((1.15,0.,.1),(0,0,0),{'settled':False},.05)
    assert c.position_acquired and after[0]==0 and after[2]<0
    c.step((1.3,0.,.1),(0,0,0),{'settled':False},.05)
    assert not c.position_acquired


def test_bounded_compensation_overcomes_actuator_deadzone_without_windup():
    c=TerminalController((.4,0,.12));pose=[0.,0.,0.];velocity=(0.,0.,0.)
    for _ in range(4000):
        cmd=c.step(pose,velocity,{'settled':not any(velocity)},.05)
        vx=cmd[0] if cmd[0]>=.12 else 0.
        wz=cmd[2] if abs(cmd[2])>=.08 else 0.
        pose[0]+=vx*math.cos(pose[2])*.05;pose[1]+=vx*math.sin(pose[2])*.05;pose[2]+=wz*.05
        velocity=(vx,0.,wz)
        assert 0<=c.forward_integral<=c.cfg.forward_integral_limit
        assert abs(c.yaw_integral)<=c.cfg.yaw_integral_limit
        if c.verified:break
    assert c.verified
    assert c.forward_integral==c.yaw_integral==0.


def test_measured_braking_shift_is_compensated_without_changing_user_goal():
    c=TerminalController((0.,0.,0.));pose=[0.,0.,0.];velocity=(0.,0.,0.);stops=0
    for _ in range(12000):
        previous=c.phase
        cmd=c.step(pose,velocity,{'settled':not any(velocity)},.05)
        vx,_,wz=cmd
        pose[0]+=vx*math.cos(pose[2])*.05;pose[1]+=vx*math.sin(pose[2])*.05;pose[2]+=wz*.05
        velocity=cmd
        if previous=='correcting' and c.phase=='settling':
            # 模拟每次收步都会后退0.18米、偏航-0.1弧度的末端。
            pose[0]-=.18;pose[2]-=.1;stops+=1
        assert c.target==(0.,0.,0.)
        assert math.hypot(*c.stop_offset[:2])<=c.cfg.max_stop_translation_m
        if c.verified:break
    assert c.verified and stops>=2
    assert math.hypot(*pose[:2])<=.25 and abs(pose[2])<=.03


def test_repeated_feedback_does_not_fake_settling_and_hysteresis():
    c=TerminalController((0,0,0))
    for _ in range(100):c.step((.05,0,0),(0,0,0),{'settled':True},0.)
    assert not c.verified
    for _ in range(30):c.step((.05,0,0),(0,0,0),{'settled':True},.05)
    assert c.verified
    # 已保持后，0.15米处仍在滞回区，不应为每一点定位抖动反复起步。
    assert c.step((.15,0,0),(0,0,0),{'settled':True},.05)==(0,0,0)
    assert c.verified
    c.step((.21,0,0),(0,0,0),{'settled':True},.05)
    assert not c.verified


def test_braking_is_not_interrupted_before_actual_stance_settles():
    c=TerminalController((0,0,0))
    assert c.step((.05,0,0),(0,0,0),{'settled':False},.05)==(0,0,0)
    assert c.phase=='settling'
    # 收步中发生偏航，仍先完成落脚；站稳后再根据实际误差纠偏。
    assert c.step((.05,0,.1),(0,0,.1),{'settled':False},.05)==(0,0,0)
    assert c.phase=='settling'
    assert any(c.step((.05,0,.1),(0,0,0),{'settled':True},.05))
    assert c.phase=='correcting'


def test_current_acceptance_window_can_settle_without_premature_restart():
    c=TerminalController((0,0,0),TerminalConfig(reacquire_fraction=1.))
    c.step((.05,0,0),(0,0,0),{'settled':False},.05)
    for _ in range(30):
        assert c.step((.21,0,.026),(0,0,0),{'settled':True},.05)==(0,0,0)
    assert c.verified
    assert any(c.step((.21,0,.031),(0,0,0),{'settled':True},.05))


def test_terminal_uses_one_policy_instead_of_blending_independent_actions(monkeypatch):
    from forest_nav_control.dual_policy import StopTransition
    monkeypatch.setenv('FOREST_NAV_TERMINAL_CONTROL','1')
    monkeypatch.setenv('FOREST_NAV_STAND_TRANSITION','direct')
    c=StopTransition();weights=[]
    for _ in range(100):weights.append(c.update((0,0,0),0,0,0,.02)['stand_weight'])
    assert set(weights)=={0.,1.}
    assert c.update((.02,0,.01),0,0,0,.02)['mode']=='walking'




def test_collision_unknown_bounds_and_coordinate_frame():
    info=NS(width=20,height=20,resolution=.1,origin=NS(position=NS(x=-1.,y=-1.)))
    grid=NS(header=NS(frame_id='odom'),info=info,data=[0]*400);cfg=TerminalConfig()
    assert trajectory_clear(grid,(0,0,0),(.18,0,0),cfg,pose_frame='odom')
    assert not trajectory_clear(grid,(0,0,0),(.18,0,0),cfg)
    for cost in (-1,99,100):
        grid.data[10*20+11]=cost
        assert not trajectory_clear(grid,(0,0,0),(.18,0,0),cfg,pose_frame='odom')
    grid.data=[0]*400
    assert not trajectory_clear(grid,(.99,0,0),(.18,0,0),cfg,pose_frame='odom')


def test_distance_growth_keeps_correcting():
    controller=TerminalController((0,0,0))
    for distance in (.9,1.01,1.3):
        command=controller.step((-distance,0,0),(0,0,0),{},.05)
        assert command[0]>0
        assert controller.phase=='correcting'
        assert not controller.verified


def test_invalid_state_fails_explicitly():
    with pytest.raises(ValueError):TerminalController((0,0,0)).step((float('nan'),0,0),(0,0,0),{},.05)
    with pytest.raises(ValueError):TerminalConfig(entry_fraction=.9,reacquire_fraction=.8)


@pytest.mark.parametrize('velocity,settled', [((.2,0,0),True), ((0,0,.1),True), ((0,0,0),False)])
def test_holding_loses_current_verification_and_must_settle_again(velocity, settled):
    c=TerminalController((0,0,0))
    for _ in range(30):c.step((0,0,0),(0,0,0),{'settled':True},.05)
    assert c.verified
    assert c.step((.01,0,0),velocity,{'settled':settled},.05)==(0,0,0)
    assert not c.verified and c.ever_verified and c.stable_time==0
    for _ in range(10):c.step((.01,0,0),(0,0,0),{'settled':True},.05)
    assert not c.verified
    for _ in range(15):c.step((.01,0,0),(0,0,0),{'settled':True},.05)
    assert c.verified


def test_terminal_turns_and_advances_beyond_old_heading_gate():
    # 目标在前方45度，旧20度门槛会强制vx=0；现在可边前进边转向。
    cfg=TerminalConfig(max_forward_mps=1.8,max_yaw_radps=1.0)
    c=TerminalController((1.,1.,0.),cfg)
    vx,vy,wz=c.step((0.,0.,0.),(0.,0.,0.),{},.05)
    assert 0.<vx<=1.8 and vy==0.
    assert .5<wz<=1.0
