"""一次真实LIO导航验收：一键入口自主走到(2,0)，只在本评估进程读取真值。

记录实际话题发布者和TF所有权，避免“看似切换，实际仍用真值”的假通过。
"""
from datetime import datetime
import argparse
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time
import numpy as np
import rclpy
from nav_msgs.msg import Odometry,OccupancyGrid
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile,DurabilityPolicy

ROOT=Path(__file__).resolve().parent;PROJECT=ROOT.parent
sys.path.insert(0,str(PROJECT/'nav2_path_tracking'))
from forest_nav_control.sensor_contract import rotation,imu_pose_from_lidar
from lio_evaluation import evaluate


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal',nargs=3,type=float,default=[2.,0.,0.],metavar=('X','Y','YAW'))
    parser.add_argument('--preset',choices=['basic','calibration','complex','stress'],default='basic')
    parser.add_argument('--require-outside',action='store_true',help='要求目标提交时在实际地图边界外')
    parser.add_argument('--settle-sim-sec',type=float,default=0.,help='Nav2成功后继续记录指定仿真秒，独立核对停稳')
    parser.add_argument('--require-unknown',action='store_true',help='必须观测到初始未知、前沿探索及已知地图增长才通过')
    parser.add_argument('--task-timeout',type=float,default=0.,help='默认0：不限制测试任务时长；正数显式限定本次测试等待')
    args=parser.parse_args()
    if not np.isfinite(args.goal).all() or not np.isfinite(args.task_timeout) or args.task_timeout<0:parser.error('目标与时限必须有效')
    if not np.isfinite(args.settle_sim_sec) or args.settle_sim_sec<0:parser.error('停稳时间必须非负且有限')
    if args.require_outside:args.require_unknown=True
    output=ROOT/'results'/('lio_navigation_'+datetime.now().strftime('%Y%m%d_%H%M%S'));output.mkdir(parents=True)
    reset_source=PROJECT/'forest_navigation/results'/args.preset/'episode_reset_diagnostic.json'
    reset_mtime_ns=reset_source.stat().st_mtime_ns if reset_source.exists() else None
    os.environ.update(FOREST_NAV_RUN_DIR=str(output/'startup'),FOREST_NAV_MISSION_RESULTS=str(output/'missions'),
        FOREST_NAV_STATE_PORT='15741',FOREST_NAV_COMMAND_PORT='15742')
    goal=args.goal;estimated=[];truth=[];localization=[];mapping=[];missions=[];bridges=[];exploration=[]
    raw_commands=[];adapted_commands=[]
    raw_maps=[];initial_snapshot={};phase=[None]
    odom_publishers=set();map_publishers=set();tf_owners={};tf_endpoints={};truth_clouds=[]
    report=dict(passed=False,goal=goal,preset=args.preset,require_unknown=args.require_unknown,require_outside=args.require_outside,
        scope='真实G1：LIO定位+CMU地图→Nav2→原策略；真值仅用于独立评估')
    log=open(output/'navigation.log','w')
    launcher=PROJECT/'forest_navigation'/('start_stress_test.sh' if args.preset=='stress' else 'start_forest_navigation.sh')
    command=['bash',str(launcher)]
    if args.preset!='stress':command+=['--preset',args.preset,'--localization','lio']
    command+=['--goal',*[str(v) for v in goal],'--headless','--startup-timeout','180','--task-timeout',str(args.task_timeout)]
    if not args.settle_sim_sec:command+=['--exit-on-goal']
    child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node('independent_lio_navigation_evaluator')
    def pose(msg,target):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        target.append([msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
    def tf_capture(msg):
        for transform in msg.transforms:
            if transform.child_frame_id in ('odom','base_link','imu_link'):
                edge=(transform.header.frame_id,transform.child_frame_id)
                tf_owners.setdefault(transform.child_frame_id,set()).add(transform.header.frame_id)
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def map_receive(msg):
        raw_maps[:]=[msg]
    def snapshot(msg,name):
        np.savez_compressed(output/name,cost=np.asarray(msg.data,dtype=np.int8).reshape(msg.info.height,msg.info.width),
            resolution=msg.info.resolution,origin=[msg.info.origin.position.x,msg.info.origin.position.y],frame_id=msg.header.frame_id)
    def exploration_receive(msg):
        data=json.loads(msg.data);exploration.append(data)
        if data.get('goal_id') and 'initial_target_unknown' in data and not initial_snapshot:
            initial_snapshot.update(unknown=bool(data['initial_target_unknown']),outside=bool(data.get('initial_target_outside')),known_cells=data.get('known_cells_initial'))
            if raw_maps:
                grid=raw_maps[-1];snapshot(grid,'initial_map.npz')
                col,row=np.floor((np.asarray(goal[:2])-[grid.info.origin.position.x,grid.info.origin.position.y])/grid.info.resolution).astype(int)
                initial_snapshot['target_cost']=int(grid.data[row*grid.info.width+col]) if 0<=col<grid.info.width and 0<=row<grid.info.height else 'outside'
        signature=(data.get('phase'),data.get('subgoal_id'),data.get('reason_code'))
        if data.get('goal_id') and signature!=phase[0]:
            phase[0]=signature
            print(json.dumps(dict(phase=data.get('phase'),reason=data.get('reason_code'),subgoal=data.get('subgoal'),
                initial_unknown=data.get('initial_target_unknown')),ensure_ascii=False),flush=True)
    subs=[node.create_subscription(Odometry,'/lio/odometry',lambda m:pose(m,estimated),100),
        node.create_subscription(Odometry,'/lidar/ground_truth_odometry',lambda m:pose(m,truth),100),
        node.create_subscription(PointCloud2,'/lidar/points_map',lambda m:truth_clouds.append(m.width),5),
        node.create_subscription(TFMessage,'/tf',tf_capture,100),node.create_subscription(TFMessage,'/tf_static',tf_capture,qos),
        node.create_subscription(OccupancyGrid,'/traversability_map',map_receive,qos),
        node.create_subscription(String,'/navigation/exploration_status',exploration_receive,20),
        node.create_subscription(Twist,'/cmd_vel',lambda m:raw_commands.append(
            (time.monotonic(),m.linear.x,m.linear.y,m.angular.z)),20),
        node.create_subscription(Twist,'/cmd_vel_adapted',lambda m:adapted_commands.append(
            (time.monotonic(),m.linear.x,m.linear.y,m.angular.z)),20)]
    for topic,target in [('/navigation/localization_status',localization),('/navigation/mapping_status',mapping),
        ('/navigation/mission_status',missions),('/navigation/bridge_status',bridges)]:
        subs.append(node.create_subscription(String,topic,lambda m,t=target:t.append(json.loads(m.data)),100))
    deadline=(time.monotonic()+args.task_timeout+280.) if args.task_timeout>0 else None;next_audit=0.;arrival_stamp=None;arrival_wall=None;controlled_stop=False
    try:
        while child.poll() is None:
            rclpy.spin_once(node,timeout_sec=.05)
            if args.settle_sim_sec and exploration and exploration[-1].get('goal_id'):
                state=exploration[-1]
                if state.get('phase')=='arrived' and truth:
                    if arrival_stamp is None:
                        arrival_stamp=truth[-1][0]
                        arrival_wall=time.monotonic()
                        print('Nav2已到达，开始独立停稳观测',flush=True)
                    if (truth[-1][0]-arrival_stamp)*1e-9>=args.settle_sim_sec:
                        controlled_stop=True;break
                elif state.get('phase') in ('blocked','fault','canceled'):
                    controlled_stop=True;break
            if deadline is not None and time.monotonic()>deadline:raise TimeoutError('LIO导航总时限超出')
            if time.monotonic()>next_audit:
                next_audit=time.monotonic()+.5
                odom_publishers.update(i.node_name for i in node.get_publishers_info_by_topic('/odom'))
                map_publishers.update(i.node_name for i in node.get_publishers_info_by_topic('/traversability_map'))
                for topic in ('/tf','/tf_static'):
                    for info in node.get_publishers_info_by_topic(topic):tf_endpoints[tuple(info.endpoint_gid)]=info.node_name
        if controlled_stop and child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=40.)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        report['startup_returncode']=child.returncode
        report['controlled_stop_after_observation']=controlled_stop
        results=[m.get('current_request',{}) or {} for m in missions]
        report['navigation_succeeded']=any(m.get('action_status')==4 for m in results)
        report['last_localization']=localization[-1] if localization else None
        report['mapping_sources']=sorted(set(m.get('source','') for m in mapping))
        report['bridge_pose_sources']=sorted(set(m.get('localization_source','') for m in bridges))
        report['odom_publishers']=sorted(odom_publishers);report['map_publishers']=sorted(map_publishers)
        report['truth_registered_cloud_messages']=len(truth_clouds)
        report['tf_parents']={child:sorted(parents) for child,parents in tf_owners.items()}
        report['tf_audit_scope']='本机Humble Python不提供逐消息发布者信息；检查父节点唯一性、实际odom发布者及真值TF禁用回归'
        metrics,paired=evaluate(estimated,truth)
        metrics['reference_40_pose_check_passed']=metrics.pop('passed')
        report['trajectory']=metrics
        if len(paired):
            truth_final=paired[-1,4:6]
            report['independent_final_goal_error_xy_m']=float(np.linalg.norm(truth_final-np.asarray(goal[:2])))
            report['independent_final_truth_pose']=truth_final.tolist()
        if localization and localization[-1].get('pose'):
            report['estimated_goal_error_m']=float(np.linalg.norm(np.asarray(localization[-1]['pose'][:2])-goal[:2]))
        thresholds=(40,.5,.75) if args.require_unknown else (15,.15,.2)
        minimum_pairs,max_rmse,max_end=thresholds
        accuracy=metrics.get('paired_poses',0)>=minimum_pairs and metrics.get('position_rmse_m',99)<max_rmse and metrics.get('end_position_error_m',99)<max_end
        report['accuracy_criteria']=dict(minimum_paired_poses=minimum_pairs,max_rmse_m=max_rmse,max_end_error_m=max_end,passed=accuracy)
        report['initial_target_unknown']=initial_snapshot.get('unknown')
        report['initial_target_cost']=initial_snapshot.get('target_cost')
        report['initial_known_cells']=initial_snapshot.get('known_cells')
        report['peak_known_cells']=max((int(m.get('known_cells',0)) for m in mapping),default=0)
        report['final_known_cells']=int(np.count_nonzero(np.asarray(raw_maps[-1].data)>=0)) if raw_maps else 0
        report['initial_target_outside']=initial_snapshot.get('outside')
        report['map_expanded']=bool(raw_maps and (output/'initial_map.npz').exists() and np.load(output/'initial_map.npz')['cost'].shape!= (raw_maps[-1].info.height,raw_maps[-1].info.width))
        report['observed_exploring']=any(e.get('phase')=='exploring' for e in exploration)
        subgoals={e['subgoal_id']:e.get('subgoal') for e in exploration if e.get('subgoal_id')}
        report['subgoals']=list(subgoals.values())
        exploration_ok=bool(initial_snapshot.get('unknown') and initial_snapshot.get('target_cost') in (-1,'outside')
            and report['observed_exploring'] and len(subgoals)>=2
            and report['final_known_cells']>(initial_snapshot.get('known_cells') or 0)+10)
        report['unknown_navigation_criteria_passed']=exploration_ok
        parking_ok=not args.settle_sim_sec
        if args.settle_sim_sec:
            # 只用真实传感器位姿差分检查实际运动，不用零指令或滤波估计冒充停稳。
            parking=dict(passed=False,required_sim_sec=args.settle_sim_sec,arrival_observed=arrival_stamp is not None)
            if arrival_stamp is not None and truth:
                actual=np.asarray(truth,dtype=np.float64)
                actual=actual[actual[:,0]>=arrival_stamp]
                window=actual[actual[:,0]>=actual[-1,0]-3e9]
                if len(window)>=8:
                    dt=np.diff(window[:,0])*1e-9
                    speeds=np.linalg.norm(np.diff(window[:,1:3],axis=0),axis=1)/dt
                    yaw=np.unwrap([np.arctan2(rotation(row[4:])[1,0],rotation(row[4:])[0,0]) for row in window])
                    angular=np.abs(np.diff(yaw))/dt
                    parking.update(duration_sim_sec=float((actual[-1,0]-arrival_stamp)*1e-9),
                        arrival_stamp_ns=int(arrival_stamp),
                        arrival_wall_monotonic=arrival_wall,
                        arrival_truth_pose=actual[0,1:3].tolist(),
                        arrival_truth_goal_error_xy_m=float(
                            np.linalg.norm(actual[0,1:3]-np.asarray(goal[:2]))),
                        settled_truth_pose=actual[-1,1:3].tolist(),
                        settled_truth_goal_error_xy_m=float(
                            np.linalg.norm(actual[-1,1:3]-np.asarray(goal[:2]))),
                        drift_from_arrival_m=float(np.linalg.norm(actual[-1,1:3]-actual[0,1:3])),
                        last_3s_xy_span_m=float(np.max(np.linalg.norm(window[:,1:3]-window[0,1:3],axis=1))),
                        last_3s_mean_speed_mps=float(np.mean(speeds)),last_3s_mean_abs_yaw_rate_radps=float(np.mean(angular)))
                    parking_ok=bool(parking['duration_sim_sec']>=args.settle_sim_sec and parking['drift_from_arrival_m']<.3
                        and parking['last_3s_xy_span_m']<.12 and parking['last_3s_mean_speed_mps']<.08
                        and parking['last_3s_mean_abs_yaw_rate_radps']<.15)
                    parking['passed']=parking_ok
                arrival_localization=[
                    row for row in localization
                    if row.get("pose") and (row.get("stamp") or 0)*1e9>=arrival_stamp
                ]
                if arrival_localization:
                    pose=arrival_localization[0]['pose']
                    parking['arrival_estimated_goal_error_xy_m']=float(
                        np.linalg.norm(np.asarray(pose[:2])-np.asarray(goal[:2])))
            report['parking']=parking
        outside_ok=not args.require_outside or (initial_snapshot.get('outside') and initial_snapshot.get('target_cost')=='outside' and report['map_expanded'])
        report['outside_navigation_criteria_passed']=outside_ok
        report['passed']=bool((child.returncode==0 or controlled_stop) and parking_ok and outside_ok and report['navigation_succeeded'] and accuracy
            and metrics.get('truth_displacement_m',0)>(.7*np.linalg.norm(goal[:2]) if args.require_unknown else 1.)
            and report.get('estimated_goal_error_m',99)<.4
            and report.get('independent_final_goal_error_xy_m',99)<.4
            and (not args.require_unknown or exploration_ok)
            and odom_publishers=={'forest_lio_ekf'} and map_publishers=={'forest_lio_navigation'}
            and report['mapping_sources']==['cmu_lio'] and report['bridge_pose_sources']==['fast_lio']
            and not truth_clouds and tf_owners.get('odom')=={'map'} and tf_owners.get('base_link')=={'odom'}
            and tf_owners.get('imu_link')=={'base_link'})
        if (reset_source.exists()
                and (reset_mtime_ns is None
                     or reset_source.stat().st_mtime_ns>reset_mtime_ns)):
            shutil.copy2(reset_source,output/'episode_reset_diagnostic.json')
            report['episode_reset_diagnostic']=json.loads(reset_source.read_text())
        if len(paired):np.savetxt(output/'aligned_trajectory.csv',paired,delimiter=',',
            header='stamp_ns,estimate_x,estimate_y,estimate_z,truth_x,truth_y,truth_z,error_m,angle_deg',comments='')
    except Exception as exc:report['error']=str(exc)
    finally:
        if child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=40.)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        np.savez_compressed(output/'trajectories.npz',estimated=np.asarray(estimated),truth_lidar=np.asarray(truth))
        if raw_maps:snapshot(raw_maps[-1],'final_map.npz')
        (output/'exploration_status.json').write_text(json.dumps(exploration,ensure_ascii=False))
        (output/'bridge_status.json').write_text(json.dumps(bridges,ensure_ascii=False))
        (output/'mapping_status.json').write_text(json.dumps(mapping,ensure_ascii=False))
        (output/'localization_status.json').write_text(json.dumps(localization,ensure_ascii=False))
        (output/'mission_status.json').write_text(json.dumps(missions,ensure_ascii=False))
        (output/'commands.json').write_text(json.dumps(
            {"raw":raw_commands,"adapted":adapted_commands},
            ensure_ascii=False)+'\n')
        (output/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown();log.close()
    print(json.dumps(dict(report=report,output=str(output)),ensure_ascii=False,indent=2),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
