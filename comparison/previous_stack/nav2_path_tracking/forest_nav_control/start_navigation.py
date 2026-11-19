"""一键启动导航服务、G1 ONNX 策略和目标执行；统一管理本次启动的子进程。"""
import argparse
import json
import math
from datetime import datetime
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT=Path(__file__).resolve().parents[1]


def simulation_environment():
    """隔离 ROS Python3.10，防止被 Isaac Python3.12 导入。"""
    env=dict(os.environ)
    env['PYTHONPATH']=str(ROOT)
    for name in ['ROS_DISTRO','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','CMAKE_PREFIX_PATH','RMW_IMPLEMENTATION']:
        env.pop(name,None)
    if 'LD_LIBRARY_PATH' in env:
        env['LD_LIBRARY_PATH']=':'.join(p for p in env['LD_LIBRARY_PATH'].split(':') if '/opt/ros/' not in p)
    return env


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal',nargs='+',type=float,default=[-7.,-2.,0.])
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--check-ready',action='store_true',help='仅检查整套启动就绪，不发送目标，完成后关闭')
    parser.add_argument('--controller',action='store_true',help='启动坐标控制面板，等待用户输入，不自动发送默认目标')
    parser.add_argument('--exit-on-goal',action='store_true',help='到达后关闭本次导航与仿真进程')
    parser.add_argument('--task-timeout',type=float,default=0.,help='默认0：目标无限等待；正数显式限制客户端等待秒数')
    parser.add_argument('--startup-timeout', type=float, default=0., help='可选启动等待时限，0为不限制')
    args=parser.parse_args()
    if args.check_ready and args.controller:
        parser.error('--check-ready不打开控制面板')
    if args.controller and args.exit_on_goal:
        parser.error('--controller 与 --exit-on-goal 不能同时使用；控制面板支持连续多个目标')
    if not math.isfinite(args.task_timeout) or args.task_timeout<0:
        parser.error('--task-timeout 必须非负，0表示无限等待')
    if not math.isfinite(args.startup_timeout) or args.startup_timeout < 0:
        parser.error('--startup-timeout 必须非负')
    if len(args.goal) not in (2,3) or not all(math.isfinite(v) for v in args.goal):
        parser.error('--goal 必须是 x y 或 x y yaw')
    run_dir = Path(os.environ.get('FOREST_NAV_RUN_DIR', str(ROOT.parent/'forest_navigation/results/startup')))
    run_dir = run_dir / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + str(os.getpid()))
    run_dir.mkdir(parents=True, exist_ok=True)
    os.environ['FOREST_NAV_READINESS_FILE'] = str(run_dir/'readiness.json')
    # 日志仅在健康状态变化/拒收时落盘，便于追查短暂断流；不改变导航许可。
    os.environ.setdefault('FOREST_NAV_DIAGNOSTICS_DIR',str(run_dir/'diagnostics'))
    def save_status(state, reason=''):
        (run_dir/'session.json').write_text(json.dumps(dict(state=state, reason=reason,
            map_mode=os.environ.get('FOREST_NAV_MAP_MODE','offline'),
            localization=os.environ.get('FOREST_NAV_LOCALIZATION','truth'),
            processes=[dict(pid=c.pid, returncode=c.poll()) for c in children],
            updated_wall_time=time.time()),ensure_ascii=False,indent=2))
    if os.environ.get('FOREST_NAV_PLANNER') == 'far':
        custom = ROOT.parent/'forest_far_navigation/install/visibility_graph_msg/local/lib/python3.10/dist-packages'
        os.environ['PYTHONPATH'] = str(custom)+os.pathsep+os.environ.get('PYTHONPATH','')
        libraries = ROOT.parent/'forest_far_navigation/install/visibility_graph_msg/lib'
        os.environ['LD_LIBRARY_PATH'] = str(libraries)+os.pathsep+os.environ.get('LD_LIBRARY_PATH','')
    children=[]
    save_status('gpu_preflight')
    if os.environ.get('FOREST_NAV_MAP_MODE')=='lidar':
        os.environ.update(FOREST_LIDAR_ENABLED='1',FOREST_LIDAR_FRAME_FILE=str(run_dir/'latest_lidar.npz'),FOREST_LIDAR_PUBLISH_CLOCK='0')
        os.environ['FOREST_LIO_OUTPUT']=str(run_dir/'lio')
    os.environ.setdefault("FOREST_NAV_DUAL_POLICY", "1")
    # 持续终点闭环默认用于激光在线导航；显式设0可回退到原停车检查流程。
    if os.environ.get('FOREST_NAV_MAP_MODE') == 'lidar':
        os.environ.setdefault('FOREST_NAV_TERMINAL_CONTROL', '1')
    env=simulation_environment()
    sim_root=Path(env.get('ISAAC_SIM_ROOT','/home/ubuntu/isaacsim-6.2.0'))
    # 先验证当前 GPU 计算环境。独立探针失败时直接停止，不让用户空等黑屏启动。
    probe=subprocess.run([str(sim_root/'python.sh'),'--no-ros-env','-c',
        'import torch; torch.cuda.init(); print("CUDA preflight OK:",torch.cuda.get_device_name(0))'],env=env)
    if probe.returncode:
        save_status('failed', 'GPU计算初始化失败')
        raise SystemExit('GPU 计算初始化失败，尚未启动导航或仿真；请先恢复本机 CUDA 运行状态。')
    def start(command,**kwargs):
        child=subprocess.Popen(command,start_new_session=True,**kwargs)
        children.append(child)
        return child
    try:
        save_status('starting')
        monitor=start(['/usr/bin/python3','-m','forest_nav_control.readiness'])
        stack=start(['bash',str(ROOT/'start_tracking.sh')])
        policy=start(['bash',str(ROOT/'start_g1_policy.sh'),'--visualizer','none' if args.headless else 'kit'],env=env)
        controller = start(['bash',str(ROOT/'open_navigation_controller.sh')]) if args.controller else None
        deadline=time.monotonic()+args.startup_timeout
        previous=None
        # 自动目标必须等统一探针全部通过。面板可提前打开，但自身ready判定仍禁止发送。
        while True:
            if controller is not None and controller.poll() is not None:
                if controller.returncode: raise RuntimeError('控制面板异常退出')
                return
            try:
                report=json.loads(Path(os.environ['FOREST_NAV_READINESS_FILE']).read_text())
            except (OSError,ValueError):
                report=dict(ready=False,reason='等待就绪探针')
            if report['ready']: break
            if report['reason'] != previous:
                print('[启动] '+report['reason'],flush=True)
                previous=report['reason']
            if args.startup_timeout>0 and time.monotonic()>deadline:
                raise TimeoutError('启动等待超时：'+report['reason'])
            time.sleep(.1)
        save_status('ready')
        print('[启动] 场景、机器人策略、地图、TF与Nav2已就绪。',flush=True)
        if args.check_ready:
            return
        if controller is not None:
            while controller.poll() is None:
                time.sleep(.2)
            if controller.returncode: raise RuntimeError('控制面板异常退出')
            return
        navigator=start(['bash',str(ROOT/'navigate.sh'),'--goal',*[str(v) for v in args.goal],'--timeout',str(args.task_timeout)])
        while navigator.poll() is None:
            time.sleep(.2)
        if navigator.returncode:
            # 旧目标可能被另一个客户端替换；不能因此关闭仍在执行新目标的整套服务。
            print(f'初始导航任务已结束，退出码 {navigator.returncode}；服务继续运行，可重新下发目标或查看任务状态。',flush=True)
            if args.exit_on_goal:
                raise RuntimeError(f'初始导航未成功，退出码 {navigator.returncode}')
        else:
            print(('FAR导航与终点停稳确认完成。' if os.environ.get('FOREST_NAV_PLANNER')=='far' else 'Nav2返回初始目标到达；物理停稳尚未验证。')+('本次进程将关闭。' if args.exit_on_goal else '服务继续运行，可下发新目标；Ctrl+C关闭本次进程。'),flush=True)
        if not args.exit_on_goal:
            while True:
                time.sleep(.2)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        save_status('failed', str(exc))
        print(f'[启动失败] {exc}；记录：{run_dir}',flush=True)
        raise
    finally:
        # 先结束目标执行和导航，再结束仿真。真实机器还必须保留自身独立急停。
        for child in reversed(children):
            try:
                os.killpg(child.pid,signal.SIGINT)
            except ProcessLookupError:
                pass
        for child in children:
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL)
                child.wait()
        # 即使启动脚本父进程已退，仍清理它所属进程组的遗留子进程。
        for child in children:
            try: os.killpg(child.pid,signal.SIGKILL)
            except ProcessLookupError: pass
        state_file=run_dir/'session.json'
        if json.loads(state_file.read_text()).get('state') != 'failed':
            save_status('stopped')


if __name__=='__main__':
    main()
