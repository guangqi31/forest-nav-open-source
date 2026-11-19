"""真实三维林地导航验收：原生Nav2 + 既有ONNX G1策略 + 实际PhysX反馈。

独立ROS域/端口。用真实odom和Action结果判定到达，不能用速度积分冒充位姿。
先做出生区短程，再走含绕障的目标；每轮保存实际轨迹和重规划路线。
"""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parent

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--smoke',action='store_true')
    parser.add_argument('--heading-check',action='store_true',help='检验正负终点朝向及长路线')
    args=parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID='187',FOREST_NAV_STATE_PORT='15471',FOREST_NAV_COMMAND_PORT='15472',ROS_LOG_DIR='/tmp/forest_g1_navigation_test_logs')
    from preflight import prepare
    prepare('basic')
    os.environ.update(FOREST_TERRAIN_SCENE=str(PROJECT/'forest_terrain_3d/generated/basic/environment.usda'),
        FOREST_NAV_MAP=str(PROJECT/'terrain_framework_baseline/results/basic/terrain_costmap.npz'),
        FOREST_NAV_SENSOR_MESH=str(ROOT/'results/basic/sensor_obstacles.npz'),FOREST_NAV_PARAMS=str(ROOT/'navigation.yaml'))
    import sys
    sys.path.insert(0,str(PROJECT/'nav2_path_tracking'))
    import rclpy
    from nav_msgs.msg import Path as RosPath
    from forest_nav_control.task_client import TaskClient
    from forest_nav_control.start_navigation import simulation_environment
    report=dict(passed=False,backend='Isaac Sim PhysX + existing G1 ONNX + Nav2 NavigateToPose',tests=[],scope='basic_scene_known_map')
    children=[];logs=[];node=None;plans=[]
    def start(name,command,env=None):
        log=(ROOT/'results'/f'g1_{name}.log').open('w');logs.append(log)
        child=subprocess.Popen(command,cwd=PROJECT/'nav2_path_tracking',env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(child)
    rclpy.init()
    try:
        start('navigation',['bash','start_tracking.sh'])
        start('policy',['bash','start_g1_policy.sh','--visualizer','none'],simulation_environment())
        node=TaskClient()
        node.plan_sub=node.create_subscription(RosPath,'/plan',lambda p:plans.append([[x.pose.position.x,x.pose.position.y] for x in p.poses]),10)
        deadline=time.monotonic()+240
        while True:
            if any(child.poll() is not None for child in children):raise RuntimeError('测试子进程提前退出，请检查policy/navigation日志')
            try:node.wait_ready(5);break
            except TimeoutError:
                if time.monotonic()>deadline:raise

        initial=node.samples[-1][1:4]
        assert math.hypot(*initial[:2])<.3, '新场景仍存在旧坐标偏移'
        sensor=json.loads((ROOT/'results/basic/sensor_runtime_verification.json').read_text());assert sensor['passed']
        # 读取运行中服务端实际生效的参数，防止只查看 YAML 而用错验收容差。
        from rcl_interfaces.srv import GetParameters
        params=node.create_client(GetParameters,'/controller_server/get_parameters')
        assert params.wait_for_service(timeout_sec=5),'无法读取 Nav2 实际参数'
        request=GetParameters.Request()
        request.names=['goal_checker.xy_goal_tolerance','goal_checker.yaw_goal_tolerance',
                       'FollowPath.min_speed_theta','FollowPath.RotateToGoal.lookahead_time',
                       'FollowPath.rotate_to_goal_heading','FollowPath.closed_loop','progress_checker.plugin']
        response=node.future(params.call_async(request),5)
        report['effective_parameters']={name:getattr(value,{1:'bool_value',2:'integer_value',3:'double_value',4:'string_value'}[value.type])
                                        for name,value in zip(request.names,response.values)}
        node.destroy_client(params)
        assert 0<report['effective_parameters']['goal_checker.yaw_goal_tolerance']<=.2,'朝向容差不符合本次验收要求'
        # 控制器内部停止阈值与用户要求的最终精度分开；不能放宽验收门槛。
        yaw_tolerance=.2
        report['required_final_yaw_tolerance_rad']=yaw_tolerance
        goals=([[1.,0.,.8],[None,None,-.8],[6.,2.,0.]] if args.heading_check
               else [[1.,0.,0.]] if args.smoke else [[1.,0.,0.],[6.,2.,0.]])
        for goal in goals:
            # 第二项在实际停稳位置只改变朝向，独立验证终点转向；
            # 不能把取消后向后折返的路径选择问题混入终点精度验收。
            if goal[0] is None:
                goal=[float(node.samples[-1][1]),float(node.samples[-1][2]),goal[2]]
            start_index=len(node.samples);plan_index=len(plans)
            print('测试目标',goal,flush=True)
            handle=node.submit(goal);outcome=node.wait_task(handle,timeout=180)
            # 即便 Action 失败也保存该目标的真实末端状态和误差。
            observed=list(node.samples[-1])
            yaw_error=abs(math.atan2(math.sin(observed[3]-goal[2]),math.cos(observed[3]-goal[2])))
            report['last_goal_observation']=dict(goal=goal,action=outcome,odom=observed,yaw_error_rad=yaw_error)
            assert outcome['action_status']==4, outcome
            # Action 成功由 Nav2 在达标瞬间返回；先记录该时刻，再单独检查
            # G1 步态收脚造成的制动滑移，不能把 0.5 秒后的位置冒充达标位置。
            arrival_pose=list(node.samples[-1][1:4])
            error=math.dist(arrival_pose[:2],goal[:2])
            # 反馈采样与 Nav2 目标检查存在一个控制周期的量化差，留 0.07 米余量。
            assert error<.32,error
            assert yaw_error<=yaw_tolerance,f'Action成功但实际朝向超差: {yaw_error} > {yaw_tolerance}'
            arrival_sim_time=node.samples[-1][0]
            settle_deadline=time.monotonic()+12
            while time.monotonic()<settle_deadline:
                node.spin_for(.1)
                if node.samples[-1][0]-arrival_sim_time<2.0:continue
                recent=[c for c in node.adapted_commands if time.monotonic()-c[0]<.3]
                # 连续半秒的实际角速度/朝向也需稳定，防止一次瞬间过零被当成停稳。
                window=[s for s in node.samples if s[0]>=node.samples[-1][0]-.5]
                stable=(len(window)>=10 and np.mean([abs(s[6]) for s in window])<.08
                        and np.mean([math.hypot(s[4],s[5]) for s in window])<.03)
                if (recent and all(max(abs(v) for v in c[1:])<1e-6 for c in recent) and stable):
                    break
            else:raise TimeoutError('到达目标后未在时限内停止输出并完成制动')
            settled_pose=list(node.samples[-1][1:4])
            settled_error=math.dist(settled_pose[:2],goal[:2])
            settled_drift=math.dist(arrival_pose[:2],settled_pose[:2])
            assert settled_error<.40,settled_error
            assert settled_drift<.30,settled_drift
            settled_yaw_error=abs(math.atan2(math.sin(settled_pose[2]-goal[2]),math.cos(settled_pose[2]-goal[2])))
            report['last_goal_observation']['settled_pose']=settled_pose
            report['last_goal_observation']['settled_yaw_error_rad']=settled_yaw_error
            assert settled_yaw_error<=yaw_tolerance,f'停稳后朝向超差: {settled_yaw_error} > {yaw_tolerance}'
            assert max(abs(math.atan2(math.sin(s[3]-goal[2]),math.cos(s[3]-goal[2]))) for s in window)<=yaw_tolerance,'停稳窗口朝向超差'
            trajectory=np.asarray(node.samples[start_index:])
            assert len(plans)>plan_index,'未观察到真实Nav2路线'
            assert (np.max(np.abs(trajectory[:,4]))>.05 or np.max(np.abs(trajectory[:,6]))>.1),'无实际行走或转向反馈'
            assert node.status['ready'] and not node.status.get('fault'),'机器人状态异常'
            test=dict(passed=True,goal=goal,final_pose=arrival_pose,position_error_m=error,
                settled_pose=settled_pose,settled_position_error_m=settled_error,
                settled_drift_m=settled_drift,yaw_error_rad=yaw_error,settled_yaw_error_rad=settled_yaw_error,
                yaw_tolerance_rad=yaw_tolerance,arrival_sim_time=arrival_sim_time,settled_sim_time=node.samples[-1][0],
                settled_mean_abs_wz_radps=float(np.mean([abs(s[6]) for s in window])),action=outcome,
                plan_count=len(plans)-plan_index,trajectory_samples=len(trajectory),
                peak_actual_vx_mps=float(np.max(np.abs(trajectory[:,4]))))
            report['tests'].append(test)
            print(json.dumps(test,ensure_ascii=False),flush=True)
            if len(report['tests'])==1:
                # 在已验证的出生区内加速再取消，检查真实仿真时间而非墙钟时间。
                from std_srvs.srv import Trigger
                cancel=node.create_client(Trigger,'/navigation/cancel')
                assert cancel.wait_for_service(timeout_sec=3)
                target=[node.samples[-1][1]+.8,node.samples[-1][2],0.]
                handle=node.submit(target);pending=handle.get_result_async()
                deadline=time.monotonic()+35
                while time.monotonic()<deadline:
                    node.spin_for(.1)
                    if len(node.samples)>2 and abs(node.samples[-1][4])>.15:break
                    if pending.done():raise AssertionError('取消测试目标在运动前已结束')
                else:raise TimeoutError('取消测试未观察到机器人行走')
                assert node.future(cancel.call_async(Trigger.Request()),5).success
                result=node.future(pending,8);assert result.status==5,result.status
                cancel_time=node.samples[-1][0];deadline=time.monotonic()+60
                while node.samples[-1][0]-cancel_time<2.0 and time.monotonic()<deadline:node.spin_for(.1)
                assert node.samples[-1][0]-cancel_time>=2.0,'没有足够的仿真时间验证停止'
                recent=[c for c in node.adapted_commands if time.monotonic()-c[0]<.3]
                assert recent and all(max(abs(v) for v in c[1:])<1e-6 for c in recent),'取消后仍输出运动指令'
                last=[s for s in node.samples if s[0]>=node.samples[-1][0]-.5]
                assert len(last)>=5
                drift=math.dist(last[0][1:3],last[-1][1:3])
                assert drift<.12,f'取消后末段0.5秒位移过大 {drift}'
                report['cancel_test']=dict(passed=True,action_status=result.status,adapted_command_zero=True,
                    elapsed_sim_s=node.samples[-1][0]-cancel_time,last_half_second_drift_m=drift,actual_velocity=node.samples[-1][4:])
        report['passed']=True
    except Exception as exc:
        report['error']=f'{type(exc).__name__}: {exc}';raise
    finally:
        name='heading' if args.heading_check else 'smoke' if args.smoke else 'navigation'
        (ROOT/'results'/f'g1_{name}_verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        if node:
            (ROOT/'results'/f'g1_{name}_trajectory.json').write_text(json.dumps(node.samples)+'\n')
            (ROOT/'results'/f'g1_{name}_plans.json').write_text(json.dumps(plans)+'\n')
            (ROOT/'results'/f'g1_{name}_commands.json').write_text(json.dumps({'raw':node.raw_commands,'adapted':node.adapted_commands})+'\n')
            node.destroy_node()
        rclpy.try_shutdown()
        for child in reversed(children):
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
        for child in children:
            try:child.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in logs:log.close()
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
if __name__=='__main__':main()
