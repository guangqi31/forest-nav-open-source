"""原生 Nav2 导航任务集成验收。末端使用运动学模型，不冒充 G1 动力学验证。

独立 ROS 域和 UDP 端口，启动真实 planner/controller/bt_navigator/速度适配器。
动态地图由本测试独占发布；每个用例都观察实际 Action 结果和末端反馈。
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
os.environ.update(ROS_DOMAIN_ID='178', FOREST_NAV_STATE_PORT='15381',
                  FOREST_NAV_COMMAND_PORT='15382', ROS_LOG_DIR='/tmp/forest_task_tests')
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid, Path as PathMessage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy
from .task_client import TaskClient

ROOT=Path(__file__).resolve().parents[1]


def main():
    children, logs = [], []
    report = {'passed':False, 'backend':'kinematic', 'tests':{}}
    node = None
    results=ROOT/'results'
    results.mkdir(exist_ok=True)
    def start(cmd,name):
        log=(results/f'tasks_{name}.log').open('w')
        logs.append(log)
        proc=subprocess.Popen(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(proc)
        return proc
    rclpy.init()
    try:
        stack=start(['bash','start_tracking.sh','publish_map:=false'],'navigation')
        simulator=start(['/usr/bin/python3','-m','forest_nav_control.kinematic_sim'],'simulator')
        node=TaskClient()
        node.task_status={}
        node.task_sub=node.create_subscription(String,'/navigation/task_status',
            lambda m:setattr(node,'task_status',json.loads(m.data)),10)
        node.plans=[]
        node.plan_sub=node.create_subscription(PathMessage,'/plan',lambda m:node.plans.append(m),10)
        pub=node.create_publisher(OccupancyGrid,'/traversability_map',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        grid=np.zeros((200,200),dtype=np.int16)
        grid[[0,-1],:]=100
        grid[:,[0,-1]]=100
        def publish():
            msg=OccupancyGrid()
            msg.header.frame_id='map'
            msg.info.width=msg.info.height=200
            msg.info.resolution=.1
            msg.info.origin.position.x=msg.info.origin.position.y=-10.
            msg.info.origin.orientation.w=1.
            msg.data=grid.ravel().tolist()
            pub.publish(msg)
            node.spin_for(1.5)
        publish()
        node.wait_ready(60)
        def check(condition,message):
            if not condition: raise AssertionError(message)
        def finish(handle, goal, timeout=60):
            result=node.wait_task(handle,timeout)
            check(result['action_status']==4, f'到达失败：{result}')
            node.spin_for(.4)
            error=math.hypot(node.samples[-1][1]-goal[0],node.samples[-1][2]-goal[1])
            check(error<.25,f'位置误差 {error}')
            return dict(result,position_error_m=error,passed=True)
        print('测试：坐标到达与周期重规划',flush=True)
        n=len(node.plans)
        goal=[-7.,-2.,0.]
        result=finish(node.submit(goal),goal)
        check(len(node.plans)-n>=2,'没有观察到周期重规划')
        report['tests']['coordinate_and_replanning']=dict(result,plan_count=len(node.plans)-n)
        print('测试：新目标替换',flush=True)
        old=node.submit([-4.,-2.,0.])
        old_result=old.get_result_async()
        node.spin_for(.8)
        goal=[node.samples[-1][1]+.6,-2.,0.]
        new=node.submit(goal)
        result=finish(new,goal)
        previous=node.future(old_result,5).status
        check(previous in (5,6), f'旧任务没有终止 {previous}')
        report['tests']['replace_goal']=dict(result,previous_action_status=previous)
        print('测试：统一取消服务与停车',flush=True)
        handle=node.submit([node.samples[-1][1]+2.,-2.,0.])
        pending=handle.get_result_async()
        node.spin_for(1.)
        cancel=node.create_client(Trigger,'/navigation/cancel')
        check(cancel.wait_for_service(timeout_sec=3),'取消服务未就绪')
        check(node.future(cancel.call_async(Trigger.Request())).success,'取消请求失败')
        check(node.future(pending,5).status==5,'取消未完成')
        node.spin_for(1.)
        check(max(abs(v) for v in node.samples[-1][4:])<.01,'取消后末端没有停下')
        report['tests']['cancel_and_stop']={'passed':True}
        print('测试：更新障碍地图后绕行',flush=True)
        x,y=node.samples[-1][1:3]
        goal=[x+2.8,y,0.]
        handle=node.submit(goal)
        node.spin_for(.5)
        # 在原直线路径前方加入方块；必须从周围绕过，不能清掉地图障碍。
        cx=int((x+1.4+10)/.1);cy=int((y+10)/.1)
        grid[cy-3:cy+4,cx-2:cx+3]=100
        before=len(node.samples)
        publish()
        result=finish(handle,goal,90)
        deviation=max(abs(s[2]-y) for s in node.samples[before:])
        check(deviation>.55,f'未观察到绕行 {deviation}')
        # 运动学末端没有物理碰撞响应，必须另查整圆占地，不能只凭到达或偏航判定。
        points=np.array([s[1:3] for s in node.samples[before:]])
        dense=[]
        for a,b in zip(points[:-1],points[1:]):
            dense.extend(np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.02))+1)))
        rows,cols=np.where((grid==100)|(grid==-1))
        low=np.column_stack((cols,rows))*.1-10.
        high=low+.1
        clearance=min(float(np.sqrt(np.square(np.maximum(np.maximum(low-point,point-high),0)).sum(axis=1)).min())
                      for point in dense)-.45-.01
        check(clearance>=0,f'完整0.45米圆形占地碰撞检查失败，保守净空 {clearance}')
        report['tests']['dynamic_obstacle_detour']=dict(result,max_lateral_deviation_m=deviation,
            body_radius_m=.45,clearance_lower_bound_m=clearance)
        node.save('tasks_detour',report['tests']['dynamic_obstacle_detour'])
        print('测试：封死通道后失败并停车',flush=True)
        x,y=node.samples[-1][1:3]
        handle=node.submit([x+2.,y,0.])
        pending=handle.get_result_async()
        node.spin_for(.5)
        cx=int((x+1.+10)/.1)
        grid[:,cx:cx+3]=100
        publish()
        status=node.future(pending,10).status
        check(status==6,f'无路可走未失败 {status}')
        node.spin_for(1.)
        check(max(abs(v) for v in node.samples[-1][4:])<.01,'路径失败后仍运动')
        report['tests']['blocked_route_stop']={'passed':True,'action_status':status}
        print('测试：未知目标拒绝',flush=True)
        grid[160:180,160:180]=-1
        publish()
        status=node.wait_task(node.submit([7.,7.,0.]),10)['action_status']
        check(status==6,f'未知目标没有失败 {status}')
        report['tests']['unknown_goal']={'passed':True,'action_status':status}
        print('测试：实际状态中断自动取消',flush=True)
        grid[:]=0;grid[[0,-1],:]=100;grid[:,[0,-1]]=100
        publish()
        x,y=node.samples[-1][1:3]
        handle=node.submit([x+2.,y,0.])
        pending=handle.get_result_async()
        node.spin_for(1.)
        os.killpg(simulator.pid,signal.SIGSTOP)
        try:
            node.spin_for(1.2)
            check(not node.status['ready'],'实际状态丢失未检出')
            check(not node.task_status.get('control_permitted',True),'故障时仍允许控制')
            check(node.task_status.get('state')=='failed','任务故障状态未上报')
        finally:
            os.killpg(simulator.pid,signal.SIGCONT)
        status=node.future(pending,8).status
        check(status in (5,6),'状态故障没有终止任务')
        node.spin_for(1.)
        check(max(abs(v) for v in node.samples[-1][4:])<.01,'反馈恢复后旧任务继续运动')
        report['tests']['state_loss_cancel']={'passed':True,'action_status':status}
        # 看门狗进程本身失去调度时，桥接端应以自己的单调时钟使许可失效。
        print('测试：监视进程失联后控制许可超时',flush=True)
        import psutil
        guard=next(p for p in psutil.Process(stack.pid).children(recursive=True)
                   if 'forest_nav_control.task_guard' in p.cmdline())
        handle=node.submit([node.samples[-1][1]+2.,node.samples[-1][2],0.])
        node.spin_for(1.)
        check(abs(node.samples[-1][4])>.05,'未建立运动状态，不能验证失联制动')
        guard.suspend()
        try:
            # 0.3秒许可超时，再给0.35/0.6秒的加速度受限制动时间。
            node.spin_for(1.3)
            check(not node.status.get('control_permitted',True),'监视节点失联后仍允许控制')
            check(max(abs(v) for v in node.samples[-1][4:])<.01,'许可超时后末端仍在运动')
        finally:
            guard.resume()
        node.future(handle.cancel_goal_async())
        report['tests']['guard_heartbeat_timeout']={'passed':True}
        report['passed']=True
        print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    except Exception as exc:
        if node is not None:
            node.save('tasks_failed',{'error':str(exc),'task_status':node.task_status})
            (results/'tasks_failed_commands.json').write_text(json.dumps(node.raw_commands))
            from rosidl_runtime_py.convert import message_to_ordereddict
            (results/'tasks_failed_plans.json').write_text(json.dumps([message_to_ordereddict(p) for p in node.plans[-3:]]))
        report['error']=f'{type(exc).__name__}: {exc}'
        raise
    finally:
        (results/'tasks_summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        for proc in children:
            if proc.poll() is None: os.killpg(proc.pid,signal.SIGINT)
        for proc in children:
            try: proc.wait(timeout=8)
            except subprocess.TimeoutExpired: os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for log in logs: log.close()


if __name__=='__main__':
    main()
