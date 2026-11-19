"""真实 Nav2+DWB 的跨进程闭环验收；默认末端为运动学模型，明确区分 G1 实测。"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from .navigation_client import NavigationClient

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend',choices=['kinematic','isaac'],default='kinematic')
    args=parser.parse_args()
    os.environ['ROS_DOMAIN_ID']='176'
    os.environ['ROS_LOG_DIR']='/tmp/forest_nav_tracking_tests'
    os.environ['FOREST_NAV_STATE_PORT']='15371'
    os.environ['FOREST_NAV_COMMAND_PORT']='15372'
    results=ROOT/'results'
    results.mkdir(exist_ok=True)
    report={'passed':False,'backend':args.backend,'tests':{}}
    children,logs=[],[]
    node=None
    def start(command,name):
        log=(results/f'{args.backend}_{name}.log').open('w')
        logs.append(log)
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,cwd=ROOT)
        children.append(child)
        return child
    rclpy.init()
    try:
        # 测试仍用原有森林代价图。Isaac 后端从同一 NPZ 生成物理障碍，不另用随机森林。
        start(['bash',str(ROOT/'start_tracking.sh')],'navigation')
        if args.backend=='kinematic':
            simulator=start(['/usr/bin/python3','-m','forest_nav_control.kinematic_sim'],'simulator')
        else:
            simulator=start(['bash',str(ROOT/'start_g1_policy.sh'),'--visualizer','none','--nav2_max_steps','10000'],'simulator')
        node=NavigationClient()
        node.wait_ready(timeout=240)
        cases=[('straight',[-6.8,-2.,0.]),('turn',[-6.8,-.8,math.pi/2])]
        # 第一轮 G1 只验短距离完整闭环；复杂转弯留给同一接口的下一轮能力标定。
        if args.backend=='isaac':
            cases=[('g1_straight',[-7.,-2.,0.])]
        for name,goal in cases:
            print(f'[闭环] {args.backend}: {name}',flush=True)
            node.samples.clear()
            node.raw_commands.clear()
            node.adapted_commands.clear()
            path=node.plan(goal)
            result=node.follow(path,timeout=180 if args.backend=='kinematic' else 600)
            if result['action_status']!=GoalStatus.STATUS_SUCCEEDED:
                raise AssertionError(f'跟踪未成功：{result}')
            node.spin_for(1.)
            final=node.samples[-1]
            error=math.hypot(final[1]-goal[0],final[2]-goal[1])
            if error>.25:
                raise AssertionError(f'目标位置误差过大：{error}')
            if not node.raw_commands or max(abs(c[1]) for c in node.raw_commands)<.05:
                raise AssertionError('没有实际输出有效前进速度')
            if not node.adapted_commands or max(abs(c[1]) for c in node.adapted_commands)<.05:
                raise AssertionError('速度没有经过适配器到达下游')
            result.update({'passed':True,'goal_xy':goal[:2],'final_pose':final[1:4],'position_error_m':error,
                           'max_raw_vx':max(abs(c[1]) for c in node.raw_commands),
                           'max_adapted_vx':max(abs(c[1]) for c in node.adapted_commands)})
            report['tests'][name]=result
            node.save(f'{args.backend}_{name}',result)
        # 路径取消必须让控制器和实际末端停下来。
        pose=node.samples[-1]
        # 取消目标选向右的已知安全通道，避免按当前朝向投射到树干上。
        goal=[pose[1]+1.2,pose[2],0.]
        path=node.plan(goal)
        result=node.follow(path,timeout=20,cancel_after=1.5)
        if result['action_status']!=GoalStatus.STATUS_CANCELED:
            raise AssertionError(f'取消状态不正确：{result}')
        # 人形仿真可能慢于实时，不能把1.5秒墙上等待当成1.5秒物理制动。
        # 观察取消后的真实仿真时间，并用连续一秒的速度与位移判断站稳，避免
        # 把人形躯干的单帧摆动误当成持续行进；窗口必须完全位于取消之后。
        canceled_sim_time=node.samples[-1][0]
        stop_deadline=time.monotonic()+30
        stopped=False
        while time.monotonic()<stop_deadline:
            node.spin_for(.1)
            if simulator.poll() is not None:
                raise AssertionError('检查停车时仿真实例已退出')
            final=node.samples[-1]
            if final[0]-canceled_sim_time<1.2:
                continue
            window=np.asarray([sample for sample in node.samples
                if sample[0]>=final[0]-1.0])
            duration=window[-1,0]-window[0,0]
            if duration<.9:
                continue
            drift=float(np.linalg.norm(window[-1,1:3]-window[0,1:3]))
            mean_speed=float(np.linalg.norm(window[:,4:6].mean(axis=0)))
            mean_yaw_rate=float(abs(window[:,6].mean()))
            output_zero=bool(node.adapted_commands and max(abs(v) for v in node.adapted_commands[-1][1:])<1e-8)
            if output_zero and drift<.05 and mean_speed<.05 and mean_yaw_rate<.08:
                stopped=True
                result.update({'stop_settling_sim_sec':final[0]-canceled_sim_time,
                    'stop_window_sim_sec':duration,'stop_window_drift_m':drift,
                    'stop_window_mean_speed_mps':mean_speed,'stop_window_mean_yaw_rate':mean_yaw_rate})
                break
            if final[0]-canceled_sim_time>10:
                break
        if not stopped:
            raise AssertionError('取消后未在规定时间内达到持续低速、低位移且输出为零的停车状态')
        result['passed']=True
        report['tests']['cancel_and_stop']=result
        # 在 controller 已空闲时注入输入，停止发送后验证适配器自己的超时归零。
        publisher=node.create_publisher(Twist,'/cmd_vel',10)
        command=Twist()
        command.linear.x=.2
        for _ in range(12):
            publisher.publish(command)
            node.spin_for(.05)
        node.spin_for(.8)
        if not node.adapted_commands or max(abs(v) for v in node.adapted_commands[-1][1:])>1e-8:
            raise AssertionError('上游停止发布后，适配器没有超时归零')
        report['tests']['adapter_timeout']={'passed':True}
        # Isaac启动器包含shell和Python子进程，给外层shell发SIGINT并不保证
        # 真正停止反馈。暂停本次独立进程组，制造确定的心跳中断；验证后恢复，
        # 再交给finally关闭，避免把仍在运行的仿真误当成“断开”。
        os.killpg(simulator.pid,signal.SIGSTOP)
        try:
            node.spin_for(1.)
            if node.status is None or node.status['ready']:
                raise AssertionError('仿真反馈停止后桥接节点没有进入失效状态')
            report['tests']['state_dropout']={'passed':True,
                'state_age_sec':node.status['state_age_sec']}
        finally:
            os.killpg(simulator.pid,signal.SIGCONT)
        report['passed']=True
        print(json.dumps(report,ensure_ascii=False),flush=True)
    except Exception as exc:
        if node is not None:
            node.save(f'{args.backend}_failed', {'error': str(exc)})
            (results/f'{args.backend}_failed_commands.json').write_text(json.dumps({'raw':node.raw_commands,'adapted':node.adapted_commands}))
        report['error']=f'{type(exc).__name__}: {exc}'
        raise
    finally:
        (results/f'{args.backend}_summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL)
                child.wait()
        for log in logs:
            log.close()


if __name__=='__main__':
    main()
