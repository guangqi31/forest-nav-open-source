"""独立LIO一键启动和有限验收。运动验收仍由原Nav2真值定位控制G1，LIO只旁路估计。"""
import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from forest_nav_control.start_navigation import simulation_environment
from lio_evaluation import evaluate

ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parent


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--check',action='store_true',help='站立采集50个定位输出后保存并退出')
    parser.add_argument('--motion-test',action='store_true',help='初始化完成后，用原导航走到(6,1.5)，评估旁路LIO')
    parser.add_argument('--timeout',type=float,default=240.,help='初始化等待墙钟秒数')
    args=parser.parse_args()
    # 此入口保持第2/3步旁路验收语义；正式LIO导航使用start_forest_navigation.sh。
    os.environ['FOREST_NAV_LOCALIZATION']='truth'
    if args.timeout<=0:parser.error('超时必须大于0')
    if not (ROOT/'install_lio/fast_lio/lib/fast_lio/fastlio_mapping').is_file():
        parser.error('请先运行build_lio.sh')
    output=ROOT/'results'/('lio_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
    output.mkdir(parents=True,exist_ok=True)
    os.environ['FOREST_LIO_OUTPUT']=str(output)
    children=[];logs=[];node=None;lock=None;estimated=[];truth=[];statuses=[];map_statuses=[]
    report=dict(passed=False,scope='FAST-LIO原始激光惯性定位→CMU地形→独立可扩展代价图，尚未接管Nav2，无回环验收',
        motion_test=args.motion_test,navigation_localization='原真值定位',output=str(output))
    def start(command,name,env=None,cwd=None):
        log=open(output/name,'w');logs.append(log)
        child=subprocess.Popen(command,env=env,cwd=cwd,start_new_session=True,stdout=log,stderr=subprocess.STDOUT)
        children.append(child);return child
    def pose_record(msg,target):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        target.append([msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
    try:
        if args.motion_test:
            # 运动模式直接管理原导航栈和策略，先等LIO初始化，再发送一个既有安全测试目标。
            lock=open('/tmp/forest_nav_live_navigation.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            os.environ.update(FOREST_NAV_MAP_MODE='lidar',FOREST_LIDAR_ENABLED='1',FOREST_LIDAR_PUBLISH_CLOCK='0',
                FOREST_LIDAR_FRAME_FILE=str(output/'latest_lidar.npz'),FOREST_NAV_DIAGNOSTICS_DIR=str(output/'diagnostics'),
                FOREST_NAV_STATE_PORT='15621',FOREST_NAV_COMMAND_PORT='15622',
                FOREST_TERRAIN_SCENE=str(PROJECT/'forest_terrain_3d/generated/basic/environment.usda'),
                FOREST_NAV_MAP=str(PROJECT/'terrain_framework_baseline/results/basic/terrain_costmap.npz'),
                FOREST_NAV_SENSOR_MESH=str(PROJECT/'forest_navigation/results/basic/sensor_obstacles.npz'),
                FOREST_NAV_PARAMS=str(ROOT/'config/navigation.yaml'))
            subprocess.run(['/usr/bin/python3',str(PROJECT/'forest_navigation/preflight.py'),'--preset','basic'],check=True)
            from forest_nav_control.task_client import TaskClient
            rclpy.init();node=TaskClient()
            start(['bash','start_tracking.sh'],'navigation.log',cwd=PROJECT/'nav2_path_tracking')
            start(['bash','start_g1_policy.sh','--visualizer','none' if args.headless else 'kit'],
                'simulation.log',simulation_environment(),PROJECT/'nav2_path_tracking')
        else:
            rclpy.init();node=rclpy.create_node('forest_lio_evaluator')
            command=['bash',str(ROOT/'start_lidar.sh'),'--timeout',str(args.timeout)]
            if args.headless or args.check:command.append('--headless')
            start(command,'sensors.log')
        start(['bash',str(ROOT/'start_lio.sh')],'lio.log')
        subs=[node.create_subscription(Odometry,'/lio/odometry',lambda m:pose_record(m,estimated),100),
            node.create_subscription(Odometry,'/lidar/ground_truth_odometry',lambda m:pose_record(m,truth),100),
            node.create_subscription(String,'/lio/status',lambda m:statuses.append(json.loads(m.data)),20),
            node.create_subscription(String,'/lio/mapping_status',lambda m:map_statuses.append(json.loads(m.data)),20)]
        print('G1+激光/IMU+独立FAST-LIO已启动，初始化期间保持零目标速度。输出：'+str(output),flush=True)
        deadline=time.monotonic()+args.timeout
        def healthy():
            for child in children:
                if child.poll() is not None:raise RuntimeError(f'子进程提前退出({child.returncode})，查看{output}')
        while not (statuses and statuses[-1]['ready'] and len(estimated)>=5
                   and map_statuses and map_statuses[-1]['ready'] and map_statuses[-1]['known_cells']>=100):
            healthy();rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()>deadline:raise TimeoutError('等待FAST-LIO有效定位超时')
        print('FAST-LIO初始化完成，已输出独立位姿和点云地图。',flush=True)
        if args.motion_test:
            node.wait_ready(timeout=60.)
            report['goal']=[6.,1.5,0.]
            report['navigation_action']=node.wait_task(node.submit(report['goal']),timeout=180.)
            node.spin_for(1.)
        elif args.check:
            while len(estimated)<50:
                healthy();rclpy.spin_once(node,timeout_sec=.05)
                if time.monotonic()>deadline:raise TimeoutError('采集定位轨迹超时')
        else:
            while True:healthy();rclpy.spin_once(node,timeout_sec=.1)
        # 使用上游公开服务保存累计点云，不从真值地图导出。
        client=node.create_client(Trigger,'/lio/map_save')
        if not client.wait_for_service(timeout_sec=3.):raise RuntimeError('地图保存服务不可用')
        future=client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=10.)
        if not future.done() or not future.result().success:raise RuntimeError('FAST-LIO地图保存失败')
        report['map_save_message']=future.result().message
        client=node.create_client(Trigger,'/lio/mapping/save')
        if not client.wait_for_service(timeout_sec=3.):raise RuntimeError('LIO代价图保存服务不可用')
        future=client.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=10.)
        if not future.done() or not future.result().success:raise RuntimeError('LIO代价图保存失败')
        report['costmaps_folder']=future.result().message
    except KeyboardInterrupt:report['interrupted']=True
    except Exception as exc:report['error']=str(exc)
    finally:
        # 先关闭运动与仿真，再关闭LIO；上游正常退出也会尝试保存未落盘的累计点云。
        for child in children:
            try:os.killpg(child.pid,signal.SIGINT)
            except ProcessLookupError:pass
        for child in children:
            try:child.wait(timeout=10.)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
            try:os.killpg(child.pid,signal.SIGKILL)
            except ProcessLookupError:pass
        metrics,paired=evaluate(estimated,truth)
        report['trajectory']=metrics;report['last_lio_status']=statuses[-1] if statuses else None
        report['last_mapping_status']=map_statuses[-1] if map_statuses else None
        report['map_bytes']=(output/'map.pcd').stat().st_size if (output/'map.pcd').exists() else 0
        report['passed']=bool(not report.get('error') and metrics['passed'] and report['map_bytes']>1000
            and report.get('costmaps_folder') and map_statuses and map_statuses[-1]['known_cells']>=100
            and (not args.motion_test or (metrics.get('truth_displacement_m',0)>2.
                and report.get('navigation_action',{}).get('action_status')==4)))
        np.savez_compressed(output/'trajectories.npz',estimated=np.asarray(estimated),truth_lidar=np.asarray(truth))
        if len(paired):np.savetxt(output/'aligned_trajectory.csv',paired,delimiter=',',
            header='stamp_ns,estimate_x,estimate_y,estimate_z,truth_x,truth_y,truth_z,error_m,angle_deg',comments='')
        (output/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        if lock:lock.close()
        for log in logs:log.close()
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    return 0 if report['passed'] or (report.get('interrupted') and not args.check and not args.motion_test) else 1


if __name__=='__main__':raise SystemExit(main())
