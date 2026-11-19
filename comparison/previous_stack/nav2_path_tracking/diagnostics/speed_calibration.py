"""G1平地速度标定：独立物理试验，不加载Nav2，不修改正式导航配置。

每个速度先加速、直行，再带速度左右转弯，最后减速停车。以真实状态评估，
指令值不冒充实际速度；首次未通过停止搜索，结果只声称给定条件下的通过档位。
失败进程继续保留Kit暂停现场，禁止自动重置后把失败算成成功。
"""
import argparse
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import time
from datetime import datetime
import numpy as np

ROOT = Path('/home/ubuntu/robot_projects/forest_nav')


def phases(speed, acceleration=.8, straight_only=False):
    """固定采样窗口是标定协议，不修改正式导航的任务等待时间。"""
    ramp = speed / acceleration
    if straight_only:
        return [('stand',4.,0.,0.,0.),('accelerate',ramp,0.,speed,0.),
                ('straight',10.,speed,speed,0.),('brake',ramp,speed,0.,0.),('stop',8.,0.,0.,0.)]
    return [('stand', 4., 0., 0., 0.),
            ('accelerate', ramp, 0., speed, 0.),
            ('straight', 8., speed, speed, 0.),
            ('left', 3., speed, speed, .3),
            ('right', 3., speed, speed, -.3),
            ('brake', ramp, speed, 0., 0.),
            ('stop', 8., 0., 0., 0.)]


def command_at(speed, elapsed, acceleration=.8, straight_only=False):
    for name, duration, start, end, yaw in phases(speed, acceleration, straight_only):
        if elapsed < duration:
            return name, [start + (end-start)*max(0., elapsed)/duration, 0., yaw]
        elapsed -= duration
    return 'done', [0., 0., 0.]


def evaluate(rows, speed, fault='', straight_only=False):
    """测量门槛预先固定；区分摔倒、跟踪不足和未站稳，不事后放宽标准。"""
    result = dict(command_mps=speed, fault=fault, stages={}, passed=False)
    issues = []
    if fault: issues.append(fault)
    for name in (('straight','stop') if straight_only else ('straight','left','right','stop')):
        part = [r for r in rows if r['phase'] == name]
        if not part:
            issues.append(name + '_missing');continue
        end = part[-1]['state']['sim_time']
        tail = [r for r in part if r['state']['sim_time'] >= end-2.]
        vx = np.asarray([r['state']['velocity'][0] for r in tail])
        wz = np.asarray([r['state']['velocity'][2] for r in tail])
        tilt = [r['state'].get('locomotion', {}).get('measured_tilt_rad', math.inf) for r in part]
        start, last = part[0]['state'], part[-1]['state']
        yaw_change = sum(math.atan2(math.sin(b['state']['pose'][2]-a['state']['pose'][2]),
                                   math.cos(b['state']['pose'][2]-a['state']['pose'][2]))
                         for a,b in zip(part,part[1:]))
        values = dict(duration_sim_sec=end-start['sim_time'],
                      mean_forward_mps=float(vx.mean()), p10_forward_mps=float(np.quantile(vx,.1)),
                      mean_yaw_radps=float(wz.mean()), yaw_change_rad=yaw_change,
                      max_tilt_rad=max(tilt), displacement_m=math.dist(start['pose'][:2],last['pose'][:2]))
        result['stages'][name] = values
        # 持续机身倾角30度属于本轮不通过，而不是宣称已跌倒；跌倒由原物理保护报告。
        if max(tilt) > math.pi/6: issues.append(name+'_excessive_tilt')
        if name == 'straight':
            if abs(float(vx.mean())-speed) > max(.10,.25*speed):issues.append('forward_tracking_insufficient')
            drift = abs(yaw_change)
            values['heading_drift_rad'] = drift
            if drift > .35:issues.append('straight_heading_drift')
        elif name in ('left','right'):
            sign = 1 if name=='left' else -1
            if sign*float(wz.mean()) < .10:issues.append(name+'_turn_tracking_insufficient')
            if float(vx.mean()) < .5*speed:issues.append(name+'_forward_lost')
        else:
            values['settled_at_end'] = bool(last.get('locomotion',{}).get('settled'))
            values['stop_displacement_m'] = values['displacement_m']
            if not values['settled_at_end']:issues.append('stop_not_settled')
    result['issues'] = issues
    result['passed'] = not issues
    # 速度增益误差单独作为控制跟踪问题保留，不能冒充摔倒速度边界。
    result['stability_passed'] = not any(x != 'forward_tracking_insufficient' for x in issues)
    result['speed_tracking_passed'] = ('forward_tracking_insufficient' not in issues) if 'straight' in result['stages'] else None
    if straight_only:
        stage=result['stages'].get('straight',{})
        result['running_stable']=bool(stage.get('duration_sim_sec',0)>=9.95
            and stage.get('max_tilt_rad',math.inf)<=math.pi/6
            and (not fault or (rows and rows[-1]['phase'] in ('brake','stop','done'))))
        result['braking_and_stop_passed']=not fault and bool(result['stages'].get('stop',{}).get('settled_at_end'))
        # 本轮标定持续直行运行极限，航向漂移、速度增益和停车另列，不冒充跌倒边界。
        result['stability_passed']=result['running_stable']
    return result


def stop_child(child):
    """正常完成后关闭本轮独立进程；物理失败的现场不走这里。"""
    if child.poll() is not None:return
    os.killpg(child.pid, signal.SIGINT)
    try:child.wait(timeout=12)
    except subprocess.TimeoutExpired:
        os.killpg(child.pid,signal.SIGTERM)
        try:child.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()


def run_trial(out, speed, seed, code, visualizer, retain_failure=True, acceleration=.8, straight_only=False):
    out.mkdir()
    env=dict(os.environ)
    for key in list(env):
        if key.startswith(('FOREST_','ROS_','AMENT_','COLCON_')):env.pop(key,None)
    # 参数只作用于隔离副本；正式接口的1.5米/秒准入范围保持不变。
    env.update(PYTHONPATH=str(code), FOREST_NAV_MAP=str(out.parent/'flat.npz'),
               FOREST_NAV_MAX_FORWARD_SPEED='3.0', FOREST_NAV_DUAL_POLICY='1',
               FOREST_NAV_TERMINAL_CONTROL='1', FOREST_NAV_STATE_PORT='15661',
               FOREST_NAV_COMMAND_PORT='15662',
               FOREST_NAV_READINESS_FILE=str(out/'readiness.json'))
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1',15661));sock.settimeout(.03)
    latest=None;seq=0;first=None;last_received=time.monotonic();began=time.monotonic()
    rows=[];fault='';previous=None;child=None;preserve=False;cmd=[0.,0.,0.]
    def send(values):
        nonlocal seq
        if latest:
            seq+=1
            packet=dict(version=1,session=latest['session'],sequence=seq,
                        sent_ns=time.monotonic_ns(),command=values)
            sock.sendto(json.dumps(packet).encode(),('127.0.0.1',15662))
    try:
        with (out/'sim.log').open('w') as log,(out/'samples.jsonl').open('w') as samples:
            child=subprocess.Popen(['bash',str(ROOT/'validate_hiking_base.sh'),'--nav2_control',
                                    '--visualizer',visualizer,'env.seed='+str(seed)],
                                   env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            (out/'process.json').write_text(json.dumps(dict(pid=child.pid,seed=seed)))
            while True:
                if child.poll() is not None:
                    fault='simulation_exited_'+str(child.returncode);break
                try:
                    raw,_=sock.recvfrom(16384);state=json.loads(raw);last_received=time.monotonic()
                    # 丢弃旧包，不能把上一轮状态或乱序包拼进本轮速度统计。
                    if latest and (state['session'] != latest['session'] or state['sequence']<=latest['sequence']):continue
                    latest=state
                    if state.get('fault'):
                        fault=state['fault'];preserve=True;send([0.,0.,0.]);break
                    if first is None and state.get('ready'):first=state['sim_time']
                    elapsed=state['sim_time']-first if first is not None else 0.
                    phase,cmd=command_at(speed,elapsed,acceleration,straight_only) if first is not None else ('waiting',[0.,0.,0.])
                    # 保留反馈对应的上一条实际应用指令；本轮计划指令单独记录，不能混称实际。
                    row=dict(phase=phase,planned_command=cmd,state=state,wall=time.monotonic())
                    rows.append(row);samples.write(json.dumps(row)+'\n')
                    if previous!=phase:
                        print(f'{speed:.2f} m/s seed={seed}: {phase}',flush=True);previous=phase
                    if phase=='done':break
                except socket.timeout:pass
                send(cmd)
                # 这是通信/启动故障保护，不是对正常导航添加总任务时限。
                now=time.monotonic()
                if (first is None and now-began>180) or (first is not None and now-last_received>5):
                    fault='startup_or_feedback_timeout';break
        result=evaluate(rows,speed,fault,straight_only)
        result.update(seed=seed,physical_simulation=bool(rows),scene='flat',preserved_failure=preserve,
                      samples=len(rows),simulator_pid=child.pid,
                      failure_scene_retained_after_trial=preserve and retain_failure)
        (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
        print(json.dumps(result,ensure_ascii=False),flush=True)
        return result
    finally:
        for _ in range(3):send([0.,0.,0.]);time.sleep(.03)
        if child and (not preserve or not retain_failure):
            if preserve:
                # 降档试验已明确授权重新开一轮；先让原有暂停机制保存冻结现场。
                # 不恢复失败机器人、不关闭跌倒保护；归档后关闭该独立进程再换档。
                time.sleep(3.)
            stop_child(child)
        sock.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--speeds',type=float,nargs='+',default=[.3,.6,.9,1.2,1.5,1.8,2.1,2.4,2.7,3.])
    p.add_argument('--repeats',type=int,default=2)
    p.add_argument('--descending',action='store_true',help='按给定降序测试，首次两轮稳定通过后停止；失败先暂停归档再开启下一轮')
    p.add_argument('--visualizer',choices=['kit','none'],default='kit')
    p.add_argument('--output',type=Path)
    p.add_argument('--acceleration',type=float,default=.8,help='本次独立测试的升降速斜率，米/秒²')
    p.add_argument('--straight-only',action='store_true',help='只标定持续直行运行极限；转向不参与，停车另列')
    a=p.parse_args()
    if not math.isfinite(a.acceleration) or not .05<=a.acceleration<=.8:p.error('加速度需在[0.05,0.8]')
    if not a.speeds or any(not math.isfinite(v) or not 0<v<=3 for v in a.speeds):p.error('测试速度须在(0,3]')
    if a.speeds!=sorted(set(a.speeds),reverse=a.descending):p.error('速度必须按搜索方向严格排序且不重复')
    if not 1<=a.repeats<=5:p.error('重复次数须为1至5')
    # 与正式一键入口共用锁，避免8GB显卡上误开第二个仿真。
    lock=open('/tmp/forest_nav_live_navigation.lock','w')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('请先关闭正式森林仿真，测试未启动')
    out=a.output or ROOT/'nav2_path_tracking/results'/('speed_calibration_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    out.mkdir(parents=True,exist_ok=False)
    code=out/'code';package=code/'forest_nav_control'
    shutil.copytree(ROOT/'nav2_path_tracking/forest_nav_control',package,ignore=shutil.ignore_patterns('__pycache__'))
    runtime=package/'isaac_runtime.py';original=runtime.read_text()
    assert original.count('terrain.size=(40.,40.)')==1
    # 扩大独立平地，防止高档位测试越出40米场地；地形仍然完全平坦。
    # 缓慢加速到3、保持10秒、再缓慢减速共行驶约120米；大平地防止边界污染结论。
    test_size='terrain.size=(320.,320.)' if a.straight_only else 'terrain.size=(160.,40.)'
    test_runtime=original.replace('terrain.size=(40.,40.)',test_size)
    if a.straight_only:
        # 直接复用IsaacLab官方平面生成器，不为零起伏大平地生成巨量Perlin噪声数组。
        # 只改结果目录的副本，接触材质、机器人和策略不变。
        test_runtime=test_runtime.replace("plane=copy.deepcopy(terrain.sub_terrains['perlin_rough'])",
            "from isaaclab.terrains.trimesh.mesh_terrains_cfg import MeshPlaneTerrainCfg\n    plane=MeshPlaneTerrainCfg(proportion=1.)")
    runtime.write_text(test_runtime)
    endpoint=package/'sim_endpoint.py';text=endpoint.read_text();assert text.count('<= 1.5:')==1
    endpoint.write_text(text.replace('<= 1.5:','<= 3.0:').replace('(0,1.5]','(0,3.0]'))
    grid=np.zeros((200,800),np.int8);grid[170:175,770:775]=100
    origin=np.array([-80.,-20.]);resolution=.2
    if a.straight_only:
        # 传感器需要非空查询网格；唯一占位障碍位于320米试验场外，不在运动区域。
        grid=np.zeros((256,256),np.int8);grid[250,250]=100
        origin=np.array([-256.,-256.]);resolution=2.
    np.savez(out/'flat.npz',cost=grid,origin=origin,resolution=resolution)
    model=Path(os.environ.get('HIKING_MODEL_ROOT','/home/ubuntu/instintlab-gpu/work/instinctlab_models/data&model/checkpoints/parkour_onboard_preview_stair'))
    (out/'protocol.json').write_text(json.dumps(dict(speeds=a.speeds,repeats=a.repeats,
        model=str(model),model_hashes={str(f.relative_to(model)):hashlib.sha256(f.read_bytes()).hexdigest() for f in (model/'exported').glob('*.onnx')},
        runtime_original_sha256=hashlib.sha256(original.encode()).hexdigest(),
        search_direction='descending' if a.descending else 'ascending',
        failure_mode='pause_archive_then_new_trial' if a.descending else 'pause_and_retain',
        test_limit_mps=3.,acceleration_mps2=a.acceleration,straight_only=a.straight_only,
        turn_command_radps=0. if a.straight_only else .3,
        straight_duration_sim_sec=10 if a.straight_only else 8,turn_each_sim_sec=0 if a.straight_only else 3,stop_duration_sim_sec=8,
        forward_tracking_tolerance='max(0.10m/s, command*25%)',max_tilt_deg=30,
        min_turn_rate_radps=.1,max_straight_heading_drift_rad=.35,
        seeds=[42+i for i in range(a.repeats)],
        assessment_version=2,stability_definition='原通过门槛保持不变，但仅forward_tracking_insufficient单独报告，不以速度增益偏差当作稳定性失败。',
        limitation='仅该模型、平地、当前接触配置及有限测试窗口；不代表林地或真机极限。'),ensure_ascii=False,indent=2))
    summary=dict(completed=False,highest_passed_command_mps=None,highest_passed_actual_mps=None,
                 first_failed_command_mps=None,trials=[],physical_failure_preserved=False,
                 search_direction='descending' if a.descending else 'ascending')
    for speed in a.speeds:
        current=[]
        for repeat in range(a.repeats):
            result=run_trial(out/f'v{speed:.2f}_r{repeat+1}',speed,42+repeat,code,a.visualizer,retain_failure=not a.descending,acceleration=a.acceleration,straight_only=a.straight_only)
            summary['trials'].append(result);current.append(result)
            if not result['stability_passed']:
                if summary['first_failed_command_mps'] is None:summary['first_failed_command_mps']=speed
                summary['physical_failure_preserved']=result['failure_scene_retained_after_trial'];break
        speed_passed=all(r['stability_passed'] for r in current) and len(current)==a.repeats
        if speed_passed:
            summary['highest_passed_command_mps']=speed
            summary['highest_passed_actual_mps']=min(r['stages']['straight']['mean_forward_mps'] for r in current)
        (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
        infrastructure_fault=bool(current[-1]['fault'] and not current[-1]['fault'].startswith(('robot_fallen','termination:')))
        if infrastructure_fault or (a.descending and speed_passed) or (not a.descending and summary['first_failed_command_mps'] is not None):break
    summary['completed']=True
    summary['search_ceiling_reached']=not a.descending and summary['first_failed_command_mps'] is None
    summary['search_floor_reached']=a.descending and summary['highest_passed_command_mps'] is None
    summary['stable_maximum_identified']=False
    summary['interpretation']='给定档位下的稳定通过记录与失败条件；未二分细化，不宣称精确物理极限。'
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    print('RESULT_DIR='+str(out),flush=True)
    if summary['physical_failure_preserved']:print('失败现场保持暂停；请手动查看并关闭，不自动退出。',flush=True)


if __name__=='__main__':main()
