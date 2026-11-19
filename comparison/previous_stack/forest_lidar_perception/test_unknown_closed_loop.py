"""一次有时限的真实Isaac未知地图导航检查，不训练、不使用离线地图选路。"""
import os,json,time,math,signal,subprocess,sys,fcntl
from pathlib import Path
from datetime import datetime
import numpy as np
ROOT=Path(__file__).resolve().parent;PROJECT=ROOT.parent
sys.path.insert(0,str(PROJECT/'nav2_path_tracking'))


def main():
    lock=open('/tmp/forest_nav_live_navigation.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    output=ROOT/'results'/('unknown_closed_loop_'+datetime.now().strftime('%Y%m%d_%H%M%S'));output.mkdir(parents=True)
    os.environ.update(ROS_DOMAIN_ID='204',ROS_LOG_DIR='/tmp/forest_unknown_closed_loop_ros',
        FOREST_NAV_MAP_MODE='lidar',FOREST_LIDAR_ENABLED='1',FOREST_LIDAR_PUBLISH_CLOCK='0',
        FOREST_LIDAR_FRAME_FILE=str(output/'latest_lidar.npz'),FOREST_NAV_MISSION_RESULTS=str(output/'missions'),FOREST_NAV_DIAGNOSTICS_DIR=str(output/'diagnostics'),
        FOREST_NAV_STATE_PORT='15621',FOREST_NAV_COMMAND_PORT='15622',
        FOREST_TERRAIN_SCENE=str(PROJECT/'forest_terrain_3d/generated/basic/environment.usda'),
        FOREST_NAV_MAP=str(PROJECT/'terrain_framework_baseline/results/basic/terrain_costmap.npz'),
        FOREST_NAV_SENSOR_MESH=str(PROJECT/'forest_navigation/results/basic/sensor_obstacles.npz'),
        FOREST_NAV_PARAMS=str(ROOT/'config/navigation.yaml'),FOREST_NAV_MISSION_TIMEOUT='160',FOREST_NAV_SUBGOAL_TIMEOUT='60')
    subprocess.run(['/usr/bin/python3',str(PROJECT/'forest_navigation/preflight.py'),'--preset','basic'],check=True,stdout=subprocess.DEVNULL)
    import rclpy
    from std_msgs.msg import String
    from forest_nav_control.task_client import TaskClient
    from forest_nav_control.goal_validation import GoalRejected
    from forest_nav_control.start_navigation import simulation_environment
    report=dict(passed=False,scope='真实G1+Isaac、在线LiDAR+CMU、双地图、explore_lite前沿、Nav2；真值定位，无SLAM')
    children=[];logs=[];node=None;statuses=[];health=[]
    health_file=open(output/'health_live.jsonl','w',buffering=1)
    def record_health(topic,msg):
        data=json.loads(msg.data)
        if topic=='/navigation/task_status':data={k:v for k,v in data.items() if k!='client_records'}
        record=dict(topic=topic,wall=time.monotonic(),data=data);health.append(record)
        health_file.write(json.dumps(record,ensure_ascii=False)+'\n')
    def start(command,name,env=None):
        log=open(output/name,'w');logs.append(log)
        child=subprocess.Popen(command,env=env,cwd=PROJECT/'nav2_path_tracking',stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(child)
    try:
        start(['bash','start_tracking.sh'],'navigation.log')
        start(['bash','start_g1_policy.sh','--visualizer','none'],'simulation.log',simulation_environment())
        rclpy.init();node=TaskClient()
        sub=node.create_subscription(String,'/navigation/exploration_status',lambda msg:statuses.append(json.loads(msg.data)),10)
        health_subs=[node.create_subscription(String,topic,lambda msg,t=topic:record_health(t,msg),10) for topic in ('/lidar/status','/lidar/diagnostics','/navigation/mapping_status','/navigation/task_status')]
        node.wait_ready(timeout=160)
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            node.spin_for(.2)
            if node.goal_validator.ready and statuses:break
        # 目标在初始CMU局部窗口以外；不人为遮蔽地图、不把未知填成可通行。
        goal=[6.,1.5,0.];report['goal']=goal
        try:node.goal_validator.check(goal);unknown=False
        except GoalRejected as exc:unknown=exc.reason=='unknown'
        report['initial_target_unknown']=unknown
        assert unknown,'预设目标初始并非未知，停止本次检查，不改地图制造未知'
        raw=node.goal_validator.grid
        report['initial_known_cells']=sum(v>=0 for v in raw.data)
        np.savez_compressed(output/'initial_map.npz',cost=np.asarray(raw.data).reshape(raw.info.height,raw.info.width),resolution=raw.info.resolution)
        print('启动一次未知目标任务：'+str(goal),flush=True)
        action=node.wait_task(node.submit(goal),timeout=175)
        node.spin_for(.5);report['action']=action;report['final_pose']=node.samples[-1][1:4]
        report['final_known_cells']=sum(v>=0 for v in node.goal_validator.grid.data)
        report['observed_exploring']=any(s.get('phase')=='exploring' for s in statuses)
        report['map_grew']=report['final_known_cells']>report['initial_known_cells']
        report['actual_displacement_m']=float(np.linalg.norm(np.array(node.samples[-1][1:3])-np.array(node.samples[0][1:3])))
        report['passed']=bool(action['action_status']==4 and report['observed_exploring'] and report['map_grew'] and report['actual_displacement_m']>.5)
    except Exception as exc:report['error']=str(exc)
    finally:
        if node:
            np.savetxt(output/'trajectory.csv',np.asarray(node.samples).reshape(-1,7),delimiter=',',header='sim_time,x,y,yaw,vx,vy,wz',comments='')
            np.savetxt(output/'commands.csv',np.asarray(node.adapted_commands).reshape(-1,4),delimiter=',',header='wall_monotonic,vx,vy,wz',comments='')
            node.destroy_node()
        (output/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        (output/'health.json').write_text(json.dumps(health,ensure_ascii=False,indent=2)+'\n')
        (output/'mission_status.json').write_text(json.dumps(statuses,ensure_ascii=False,indent=2)+'\n')
        if rclpy.ok():rclpy.shutdown()
        for child in reversed(children):
            try:os.killpg(child.pid,signal.SIGINT)
            except ProcessLookupError:pass
        for child in children:
            try:child.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        for child in children:
            try:os.killpg(child.pid,signal.SIGKILL)
            except ProcessLookupError:pass
        health_file.close()
        for log in logs:log.close()
    print(json.dumps(dict(report=report,output=str(output)),ensure_ascii=False),flush=True)
    return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
