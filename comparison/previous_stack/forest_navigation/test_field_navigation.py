"""第二阶段真实G1验收：树干/倒木绕行、目标替换、取消、不可达与复杂地形。

使用已有Nav2与ONNX策略；每个场景从实际仿真出生点开始。报告始终保存实际
反馈和失败原因，不把规划通过或运动学模型当成G1行走通过。
"""
import argparse,json,math,os,signal,subprocess,sys,time
from pathlib import Path
import numpy as np
from field_checks import FieldMap,angle_error
ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parent
SCENARIOS={
    'tree':dict(preset='basic',goal=[.2,-4.8],object_id='tree_027'),
    # 原(5.5,11)为13.6度下坡，零命令后滑移，失败保留在历史结果。
    # 新终点仍在同一倒木后方，中心坡度约0.42度，且整圆占地留有余量。
    'log':dict(preset='basic',goal=[6.95,12.85],object_id='log_004'),
    'tasks':dict(preset='basic'),
    'complex':dict(preset='complex',goal=[-5.,-1.]),
}


def run(args):
    case=SCENARIOS[args.scenario];preset=case['preset']
    output=args.output or ROOT/'results'/('field_'+args.scenario+'_'+time.strftime('%Y%m%d_%H%M%S'))
    output.mkdir(parents=True,exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID='188',FOREST_NAV_STATE_PORT='15501',FOREST_NAV_COMMAND_PORT='15502',
                      ROS_LOG_DIR='/tmp/forest_field_navigation_ros_logs')
    from preflight import prepare
    prepare(preset)
    os.environ.update(FOREST_TERRAIN_SCENE=str(PROJECT/'forest_terrain_3d/generated'/preset/'environment.usda'),
        FOREST_NAV_MAP=str(PROJECT/'terrain_framework_baseline/results'/preset/'terrain_costmap.npz'),
        FOREST_NAV_SENSOR_MESH=str(ROOT/'results'/preset/'sensor_obstacles.npz'),FOREST_NAV_PARAMS=str(ROOT/'navigation.yaml'))
    sys.path.insert(0,str(PROJECT/'nav2_path_tracking'))
    import rclpy
    from nav_msgs.msg import Path as RosPath
    from std_srvs.srv import Trigger
    from forest_nav_control.task_client import TaskClient
    from forest_nav_control.start_navigation import simulation_environment
    import shutil,hashlib
    shutil.copy2(ROOT/'navigation.yaml',output/'navigation.yaml')
    report=dict(passed=False,scenario=args.scenario,preset=preset,
                backend='real Isaac Sim PhysX + existing G1 ONNX + native Nav2',tests={},
                source_sha256={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
                               for name in ['navigation.yaml','test_field_navigation.py','field_checks.py']})
    for name in report['source_sha256']:shutil.copy2(ROOT/name,output/name)
    sources=PROJECT/'nav2_path_tracking/forest_nav_control'
    report['goal_entry_source_sha256']={name:hashlib.sha256((sources/name).read_bytes()).hexdigest()
        for name in ['goal_validation.py','task_client.py','coordinate_controller.py']}
    for name in report['goal_entry_source_sha256']:shutil.copy2(sources/name,output/name)
    import yaml
    cfg=yaml.safe_load((ROOT/'navigation.yaml').read_text())
    cm=cfg['global_costmap']['global_costmap']['ros__parameters']
    report['planning_footprint']=dict(robot_radius_m=cm['robot_radius'],padding_m=cm['footprint_padding'],
                                      independently_checked_body_radius_m=.45)
    children=[];logs=[];node=None;plans=[];field=FieldMap(PROJECT,preset)
    def start(name,command,env=None):
        log=(output/(name+'.log')).open('w');logs.append(log)
        child=subprocess.Popen(command,cwd=PROJECT/'nav2_path_tracking',env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(child)
    def check_health():
        if any(child.poll() is not None for child in children):raise RuntimeError('测试子进程提前退出')
        if node.status and (not node.status['ready'] or node.status.get('fault')):raise RuntimeError(f'真实机器人状态异常 {node.status}')
    def cancel_current():
        service=node.create_client(Trigger,'/navigation/cancel')
        try:
            assert service.wait_for_service(timeout_sec=3),'取消服务未就绪'
            assert node.future(service.call_async(Trigger.Request()),5).success,'取消请求未接受'
        finally:node.destroy_client(service)
    def wait_moving(pending,min_distance=.15,timeout=40):
        initial=list(node.samples[-1]);deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            node.spin_for(.1);check_health()
            if pending.done():raise AssertionError('目标在建立运动前已结束')
            if math.dist(initial[1:3],node.samples[-1][1:3])>=min_distance and abs(node.samples[-1][4])>.1:return
        raise TimeoutError('未观察到足够的真实前进运动')
    def settled(timeout=25):
        begin=node.samples[-1][0];deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            node.spin_for(.1);check_health()
            if node.samples[-1][0]-begin<2.:continue
            window=[s for s in node.samples if s[0]>=node.samples[-1][0]-.5]
            commands=[c for c in node.adapted_commands if time.monotonic()-c[0]<.3]
            if (len(window)>=10 and commands and all(max(abs(v) for v in c[1:])<1e-6 for c in commands)
                and np.mean([math.hypot(s[4],s[5]) for s in window])<.05
                and np.mean([abs(s[6]) for s in window])<.08):
                drift=math.dist(window[0][1:3],window[-1][1:3]);assert drift<.08,drift
                return dict(sim_time=node.samples[-1][0],pose=list(node.samples[-1][1:4]),
                            last_half_second_drift_m=drift,adapted_command_zero=True,
                            mean_abs_wz_radps=float(np.mean([abs(s[6]) for s in window])))
        raise TimeoutError('输出未归零或真实机器人没有停稳')
    def finish(handle,goal,timeout=240):
        outcome=node.wait_task(handle,timeout=timeout)
        report['last_goal_observation']=dict(goal=goal,action=outcome,pose=node.samples[-1][1:4])
        assert outcome['action_status']==4,outcome
        arrival=list(node.samples[-1][1:4]);stop=settled()
        error=math.dist(stop['pose'][:2],goal[:2]);yaw=angle_error(stop['pose'][2],goal[2])
        assert error<.4,f'停稳位置误差 {error}'
        assert yaw<=.2,f'停稳朝向误差 {yaw}'
        return dict(passed=True,goal=goal,action=outcome,arrival_pose=arrival,stop=stop,
                    settled_position_error_m=error,settled_yaw_error_rad=yaw)
    def submit_native_for_failure_test(goal):
        # 独立核对原生Nav2失败行为，测试显式绕过项目入口准入；不向生产暴露绕过开关。
        from nav2_msgs.action import NavigateToPose
        from geometry_msgs.msg import PoseStamped
        request=NavigateToPose.Goal();request.pose=PoseStamped()
        request.pose.header.frame_id='map';request.pose.header.stamp=node.get_clock().now().to_msg()
        request.pose.pose.position.x,request.pose.pose.position.y=map(float,goal[:2])
        request.pose.pose.orientation.z=math.sin(goal[2]/2);request.pose.pose.orientation.w=math.cos(goal[2]/2)
        handle=node.future(node.navigator.send_goal_async(request,feedback_callback=node.on_feedback))
        assert handle.accepted,'原生Nav2拒绝接收测试任务'
        return handle
    rclpy.init()
    try:
        start('navigation',['bash','start_tracking.sh'])
        start('policy',['bash','start_g1_policy.sh','--visualizer','none'],simulation_environment())
        node=TaskClient()
        def on_plan(msg):
            plans.append(dict(wall_monotonic=time.monotonic(),sim_time=node.get_clock().now().nanoseconds/1e9,
                              points=[[p.pose.position.x,p.pose.position.y] for p in msg.poses]))
        node.plan_sub=node.create_subscription(RosPath,'/plan',on_plan,10)
        deadline=time.monotonic()+240
        while True:
            if any(c.poll() is not None for c in children):raise RuntimeError('导航/策略启动失败')
            try:node.wait_ready(5);break
            except TimeoutError:
                if time.monotonic()>deadline:raise
        assert math.hypot(*node.samples[-1][1:3])<.3,'出生点异常'
        if 'goal' in case:
            xy=case['goal'];goal=[*xy,math.atan2(xy[1],xy[0])]
            initial=list(node.samples[-1][1:4]);begin=len(node.samples)
            if 'object_id' in case:
                direct=field.direct_obstruction(initial,goal,case['object_id'])
                report['direct_obstruction']=direct
                assert direct['straight_intersects_object'] and direct['straight_blocked_samples']>0,'直线没有被指定真实物体阻断'
            path=node.plan(goal)
            points=[[p.pose.position.x,p.pose.position.y] for p in path.poses]
            planned=field.check_path(points,initial,goal);report['planned_path_check']=planned
            report['goal_terrain']=field.goal_details(goal)
            assert planned['blocked_or_unknown_samples']==0 and planned['swept_clearance_lower_bound_m']>=0,planned
            if 'object_id' in case:assert planned['max_lateral_deviation_m']>.5,'规划未形成明确绕行'
            print('开始真实G1导航',args.scenario,goal,flush=True)
            result=finish(node.submit(goal),goal)
            actual=field.check_path([s[1:3] for s in node.samples[begin:]],initial,goal)
            result['actual_path_check']=actual
            report['tests']['detour' if 'object_id' in case else 'terrain_navigation']=result
            assert actual['blocked_or_unknown_samples']==0 and actual['swept_clearance_lower_bound_m']>=0,actual
            if 'object_id' in case:assert actual['max_lateral_deviation_m']>.5,'实际机器人没有形成明确绕行'
        else:
            # 在出生区内先替换运动中的目标，再发送下一目标，避免依赖预设假位姿。
            print('测试项目入口拒绝不宜停稳的目标',flush=True)
            from forest_nav_control.goal_validation import GoalRejected
            rejected=[]
            for bad,reason in [([5.5,11.,0.],'stopping_cost'),([17.,0.,0.],'outside')]:
                initial=list(node.samples[-1][1:4])
                try:node.submit(bad)
                except GoalRejected as exc:
                    assert exc.reason==reason,(exc.reason,reason)
                    rejected.append(dict(goal=bad,reason=exc.reason,message=str(exc)))
                else:raise AssertionError('客户端未拒绝未验证目标')
                stop=settled();assert math.dist(initial[:2],stop['pose'][:2])<.08
            # 不创建Tk窗口也调用面板实际使用的同一个发送方法，核对前后端准入一致。
            from forest_nav_control.coordinate_controller import CoordinateController
            panel=CoordinateController()
            try:
                deadline=time.monotonic()+8
                while time.monotonic()<deadline and not panel.ready:
                    panel.poll();node.spin_for(.02)
                assert panel.ready,'控制面板ROS接口未就绪'
                try:panel.send_target(5.5,11.,0.)
                except GoalRejected as exc:assert exc.reason=='stopping_cost'
                else:raise AssertionError('面板入口没有拒绝高代价终点')
                assert panel.pending_goal is None,'拒绝后仍发送了Action'
            finally:panel.destroy_node()
            report['tests']['goal_admission']=dict(passed=True,max_stopping_cost=node.goal_validator.max_cost,
                client_rejections=rejected,panel_send_target_rejected=True)
            print('测试运动中换目标',flush=True)
            old=node.submit([1.8,0.,0.]);old_result=old.get_result_async()
            wait_moving(old_result,min_distance=.2)
            goal=[1.2,.25,.2];new=node.submit(goal)
            result=finish(new,goal)
            previous=node.future(old_result,5).status
            assert previous in (5,6),f'旧任务未终止 {previous}'
            report['tests']['replace_goal']=dict(result,previous_action_status=previous)
            print('测试到达后继续下一个目标',flush=True)
            goal=[1.6,.5,.2]
            report['tests']['next_goal']=finish(node.submit(goal),goal)
            print('测试停稳后向后方目标折返',flush=True)
            goal=[.6,.25,-2.9]
            report['tests']['reverse_target']=finish(node.submit(goal),goal)
            print('测试运动中取消',flush=True)
            x,y,yaw=node.samples[-1][1:4]
            goal=[x+math.cos(yaw),y+math.sin(yaw),yaw]
            handle=node.submit(goal);pending=handle.get_result_async();wait_moving(pending)
            cancel_current();status=node.future(pending,8).status
            assert status==5,status
            report['tests']['cancel_stop']=dict(passed=True,action_status=status,stop=settled())
            print('测试障碍内部和越界目标失败后停稳',flush=True)
            obj=min((o for o in field.manifest['objects'] if o['kind']=='tree'),key=lambda o:math.hypot(o['x'],o['y']))
            for name,goal in [('occupied_goal',[obj['x'],obj['y'],0.]),('outside_goal',[17.,0.,0.])]:
                initial=list(node.samples[-1][1:4]);outcome=node.wait_task(submit_native_for_failure_test(goal),timeout=25)
                assert outcome['action_status']==6,outcome
                stop=settled();drift=math.dist(initial[:2],stop['pose'][:2])
                assert drift<.08,f'失败目标仍引起显著运动 {drift}'
                report['tests'][name]=dict(passed=True,goal=goal,action=outcome,stop=stop,displacement_m=drift)
        check_health()
        audit=json.loads((ROOT/'results'/preset/'safety_runtime_verification.json').read_text())
        report['physical_audit']=audit
        assert audit['falls_observed']==0 and audit['max_nonfoot_contact_n']<5.,audit
        assert audit['max_measured_radius_m']<=.45,audit
        # 对整个测试会话的实际平移轨迹检查整圆占地；抽样包络单独报告。
        all_path=field.check_path([s[1:3] for s in node.samples],node.samples[0][1:3],node.samples[-1][1:3])
        report['whole_session_path_check']=all_path
        assert all_path['blocked_or_unknown_samples']==0 and all_path['swept_clearance_lower_bound_m']>=0,all_path
        report['passed']=True
    except Exception as exc:
        report['error']=f'{type(exc).__name__}: {exc}'
        if node:
            report['failure_pose']=list(node.samples[-1]) if node.samples else None
            try:cancel_current()
            except Exception:pass
        raise
    finally:
        (output/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        if node:
            (output/'trajectory.json').write_text(json.dumps(node.samples)+'\n')
            (output/'plans.json').write_text(json.dumps(plans)+'\n')
            (output/'commands.json').write_text(json.dumps(dict(raw=node.raw_commands,adapted=node.adapted_commands))+'\n')
            node.destroy_node()
        rclpy.try_shutdown()
        for child in reversed(children):
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
        for child in children:
            try:child.wait(timeout=15)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        for log in logs:log.close()
        for name in ['preflight.json','sensor_runtime_verification.json','safety_runtime_verification.json']:
            source=ROOT/'results'/preset/name
            if source.exists():shutil.copy2(source,output/name)
        print('结果目录',output,flush=True)
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',choices=list(SCENARIOS),required=True)
    parser.add_argument('--output',type=Path)
    run(parser.parse_args())
