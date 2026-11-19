"""真实操作 Tk 面板控件并验证 Nav2 闭环；可切换运动学或 Isaac G1 末端。

不直接调用后端 send_target 代替点击：测试把文字填入输入框，再调用按钮的
invoke，走与用户点击相同的输入校验、角度转换和异步 Action 回调链路。
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import tkinter as tk
import rclpy
from .coordinate_controller import CoordinateController, ControllerPanel

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend',choices=['kinematic','isaac'],default='kinematic')
    args=parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID='180',FOREST_NAV_STATE_PORT='15401',
        FOREST_NAV_COMMAND_PORT='15402',ROS_LOG_DIR='/tmp/forest_controller_tests')
    children,logs=[],[]
    report={'passed':False,'backend':args.backend,'tests':{}}
    node=root=panel=None
    results=ROOT/'results'
    results.mkdir(exist_ok=True)
    def start(cmd,name,env=None):
        log=(results/f'controller_{args.backend}_{name}.log').open('w')
        logs.append(log)
        child=subprocess.Popen(cmd,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(child)
        return child
    rclpy.init()
    samples=[]
    def pump_until(condition,timeout):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            for p in children:
                if p.poll() is not None: raise RuntimeError(f'测试进程提前退出 {p.args}: {p.returncode}')
            root.update()
            if node.pose is not None:
                samples.append([time.monotonic(),*node.pose,*(node.velocity or (0.,0.))])
            if condition(): return
            time.sleep(.01)
        raise TimeoutError(f'控制面板验收超时：{node.event}；任务状态 {node.task}')
    def click_goal(x,y,yaw):
        panel.fill(x,y,yaw)
        panel.go_button.invoke()
    try:
        start(['bash','start_tracking.sh'],'navigation')
        if args.backend=='kinematic':
            start(['/usr/bin/python3','-m','forest_nav_control.kinematic_sim'],'simulator')
        else:
            from .start_navigation import simulation_environment
            start(['bash','start_g1_policy.sh','--visualizer','kit'], 'simulator',simulation_environment())
        node=CoordinateController()
        root=tk.Tk()
        panel=ControllerPanel(root,node)
        root.geometry('+40+80')
        root.update()
        # 启动阶段禁止点击发送；用户无需在仿真窗口里手动启动策略。
        if str(panel.go_button['state'])!='disabled': raise AssertionError('未就绪时发送按钮仍可用')
        pump_until(lambda:node.ready and str(panel.go_button['state'])=='normal',240)
        report['tests']['wait_until_ready']={'passed':True,'initial_pose':node.pose}
        panel.values[0].set('不是数字')
        panel.go_button.invoke()
        if node.pending_goal is not None or '有效数字' not in node.event: raise AssertionError('非法输入没有在界面拦截')
        report['tests']['input_validation']={'passed':True}
        begin=node.pose
        first=len(samples)
        click_goal(-7.,-2.,0.)
        print('已通过面板点击前往目标，等待实际机器人到达',flush=True)
        pump_until(lambda:node.last_result is not None,180)
        if node.last_result!=4: raise AssertionError(f'面板目标未成功：{node.last_result} {node.event}')
        error=math.dist(node.pose[:2],(-7.,-2.))
        movement=math.dist(begin[:2],node.pose[:2])
        if error>.25 or movement<.5: raise AssertionError(f'实际末端运动不符合目标：误差 {error}，位移 {movement}')
        peak=max(abs(s[4]) for s in samples[first:])
        if peak<.05: raise AssertionError('没有实际速度反馈，不能证明策略被驱动')
        report['tests']['button_drives_robot']={'passed':True,'goal':[-7.,-2.,0.],
            'final_pose':node.pose,'position_error_m':error,'actual_displacement_m':movement,'peak_measured_vx_mps':peak}
        root.lift()
        root.update()
        from PIL import ImageGrab
        bbox=(root.winfo_rootx(),root.winfo_rooty(),root.winfo_rootx()+root.winfo_width(),root.winfo_rooty()+root.winfo_height())
        ImageGrab.grab(bbox=bbox).save(results/f'controller_{args.backend}.png')
        if args.backend=='kinematic':
            # 第二轮目标不重启任何进程；再发更近目标，验证替换与旧结果隔离。
            click_goal(-4.,-2.,0.)
            pump_until(lambda:node.goal_handle is not None,5)
            started=time.monotonic()
            pump_until(lambda:time.monotonic()-started>.6,2)
            goal=(node.pose[0]+.6,node.pose[1])
            click_goal(*goal,0.)
            pump_until(lambda:node.last_result is not None,30)
            if node.last_result!=4 or math.dist(node.pose[:2],goal)>.25: raise AssertionError('面板替换目标未到达')
            report['tests']['replace_from_panel']={'passed':True}
            click_goal(node.pose[0]+.6,node.pose[1],0.)
            pump_until(lambda:node.velocity is not None and node.velocity[0]>.1,8)
            panel.stop_button.invoke()
            pump_until(lambda:node.task.get('state')=='canceled' and abs(node.velocity[0])<.01,8)
            report['tests']['stop_button']={'passed':True}
        report['passed']=True
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    except Exception as exc:
        report['error']=f'{type(exc).__name__}: {exc}'
        raise
    finally:
        (results/f'controller_{args.backend}_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        (results/f'controller_{args.backend}_trajectory.json').write_text(json.dumps(samples)+'\n')
        if panel is not None: panel.close()
        if node is not None: node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for p in children:
            if p.poll() is None: os.killpg(p.pid,signal.SIGINT)
        for p in children:
            try:p.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for log in logs:log.close()


if __name__=='__main__':
    main()
