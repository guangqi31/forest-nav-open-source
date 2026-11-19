"""独立诊断：真实G1物理，固定速度阶跃；不启动定位、规划或位置控制器。"""
import os,json,time,socket,subprocess,signal,math,argparse,hashlib
from datetime import datetime
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--variant',choices=['before','after'],default='after')
p.add_argument('--runtime-file',type=Path,help='before对比所需的备份isaac_runtime.py')
p.add_argument('--output',type=Path,help='诊断结果目录，每轮使用独立目录')
a=p.parse_args()
if a.variant=='before' and (a.runtime_file is None or not a.runtime_file.is_file()):p.error('before必须提供有效的--runtime-file备份')
root=Path(__file__).resolve().parents[2]
out=a.output or root/'nav2_path_tracking/results'/('lowlevel_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+a.variant)
out.mkdir(parents=True,exist_ok=False)
source=a.runtime_file if a.variant=='before' else root/'nav2_path_tracking/forest_nav_control/isaac_runtime.py'
(out/'metadata.json').write_text(json.dumps(dict(runtime_source=str(source),sha256=hashlib.sha256(source.read_bytes()).hexdigest(),seed=42,diagnostic_only=True,phases='4秒站立/6秒直行/8秒收步/6秒转向/8秒收步，均为仿真时间',truth_used='仅隔离评估，不用于生产导航定位'),ensure_ascii=False,indent=2))
# 传感器要求非空障碍网格，远处固定障碍不在本次运动区域。
grid=np.zeros((200,200),np.int8);grid[170:175,170:175]=100
np.savez(out/'flat.npz',cost=grid,origin=np.array([-20.,-20.]),resolution=np.array(.2))
env=dict(os.environ)
for key in list(env):
    if key.startswith(('FOREST_','ROS_','AMENT_','COLCON_')): env.pop(key,None)
env.update(PYTHONPATH=str(out/'code') if a.variant=='before' else str(root/'nav2_path_tracking'),FOREST_NAV_MAP=str(out/'flat.npz'),FOREST_NAV_DUAL_POLICY='1',FOREST_NAV_TERMINAL_CONTROL='1',FOREST_NAV_STATE_PORT='15561',FOREST_NAV_COMMAND_PORT='15562')
if a.variant=='before':
    import shutil
    pkg=out/'code/forest_nav_control';shutil.copytree(root/'nav2_path_tracking/forest_nav_control',pkg,dirs_exist_ok=True)
    shutil.copy2(a.runtime_file,pkg/'isaac_runtime.py')
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('127.0.0.1',15561));sock.settimeout(.025)
# 各阶段长度为预先定义的仿真采样窗口，不是导航任务超时。
phases=[('initial_stand',4,(0,0,0)),('forward',6,(.3,0,0)),('stop_forward',8,(0,0,0)),('turn',6,(0,0,.2)),('stop_turn',8,(0,0,0))]
child=None;latest=None;seq=0;rows=[];first=None;phase_prev=None
try:
    with (out/'sim.log').open('w') as log,(out/'samples.jsonl').open('w') as samples:
        child=subprocess.Popen(['bash',str(root/'validate_hiking_base.sh'),'--nav2_control','--visualizer','none'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        while child.poll() is None:
            try:
                data,_=sock.recvfrom(8192);state=json.loads(data)
                latest=state
                if state.get('fault'):raise RuntimeError(state['fault'])
                if state.get('ready') and first is None:first=state['sim_time'];print(a.variant,'ready',flush=True)
                elapsed=state['sim_time']-first if first is not None else -1
                end=0;phase='waiting';cmd=(0,0,0)
                if elapsed>=0:
                    for name,duration,command in phases:
                        end+=duration
                        if elapsed<end:phase=name;cmd=command;break
                    else:break
                if phase!=phase_prev:print(a.variant,phase,'sim=',state['sim_time'],flush=True);phase_prev=phase
                row=dict(phase=phase,command=cmd,state=state,wall=time.monotonic());rows.append(row);samples.write(json.dumps(row)+'\n')
            except socket.timeout:pass
            if latest:
                seq+=1;packet=dict(version=1,session=latest['session'],sequence=seq,sent_ns=time.monotonic_ns(),command=cmd)
                sock.sendto(json.dumps(packet).encode(),('127.0.0.1',15562))
        if child.poll() is not None:raise RuntimeError('仿真提前退出:'+str(child.returncode))
    result=dict(variant=a.variant,physical_simulation=True,goal_parking_passed=False,stages={})
    for name,_,_ in phases:
        stage=[r for r in rows if r['phase']==name]
        if not stage:continue
        start,last=stage[0]['state'],stage[-1]['state'];duration=last['sim_time']-start['sim_time']
        tail=[r['state'] for r in stage if r['state']['sim_time']>=last['sim_time']-2]
        settled=(next((r['state']['sim_time']-start['sim_time'] for r in stage if r['state'].get('locomotion',{}).get('settled')),None) if name.startswith('stop_') else None)
        result['stages'][name]=dict(duration_sim_sec=duration,displacement_m=math.dist(start['pose'][:2],last['pose'][:2]),yaw_change_rad=math.atan2(math.sin(last['pose'][2]-start['pose'][2]),math.cos(last['pose'][2]-start['pose'][2])),tail_speed_mps=sum(math.hypot(*r['velocity'][:2]) for r in tail)/len(tail),tail_yaw_rate_radps=sum(r['velocity'][2] for r in tail)/len(tail),settled_after_sim_sec=settled,last_locomotion=last.get('locomotion'))
    (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False),flush=True)
except Exception as ex:
    (out/'error.txt').write_text(str(ex));raise
finally:
    if latest:
        for _ in range(5):
            seq+=1;sock.sendto(json.dumps(dict(version=1,session=latest['session'],sequence=seq,sent_ns=time.monotonic_ns(),command=[0,0,0])).encode(),('127.0.0.1',15562));time.sleep(.03)
    if child and child.poll() is None:
        os.killpg(child.pid,signal.SIGINT)
        try:child.wait(timeout=15)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
    sock.close()
