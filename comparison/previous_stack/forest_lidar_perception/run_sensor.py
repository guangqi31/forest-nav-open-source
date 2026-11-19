"""雷达独立启动入口：默认显示G1；--check检查真实激光/IMU与同源时间后自动关闭。"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from datetime import datetime
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2,Imu
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from sensor_msgs_py import point_cloud2
from forest_nav_control.start_navigation import simulation_environment

ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parent


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preset',choices=('basic','complex','calibration'),default='basic')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--timeout',type=float,default=240.)
    args=parser.parse_args()
    if args.timeout<=0:parser.error('--timeout必须为正数')
    # 同一启动锁防止两个策略进程占用同一端口或GPU；锁只保留在父进程中。
    import fcntl
    lock=open('/tmp/forest_nav_live_navigation.lock','w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    run=ROOT/'results'/datetime.now().strftime('%Y%m%d_%H%M%S')
    run.mkdir(parents=True,exist_ok=True)
    subprocess.run(['/usr/bin/python3',str(PROJECT/'forest_navigation/preflight.py'),'--preset',args.preset],check=True)
    os.environ.update(FOREST_TERRAIN_SCENE=str(PROJECT/f'forest_terrain_3d/generated/{args.preset}/environment.usda'),
        FOREST_NAV_MAP=str(PROJECT/f'terrain_framework_baseline/results/{args.preset}/terrain_costmap.npz'),
        FOREST_NAV_SENSOR_MESH=str(PROJECT/f'forest_navigation/results/{args.preset}/sensor_obstacles.npz'),
        FOREST_NAV_MAP_MODE='offline',FOREST_LIDAR_ENABLED='1',
        FOREST_LIDAR_FRAME_FILE=str(run/'latest_frame.npz'),FOREST_LIDAR_PUBLISH_CLOCK='1',
        FOREST_NAV_STATE_PORT='15561',FOREST_NAV_COMMAND_PORT='15562')
    children=[];logs=[];node=None
    report=dict(passed=False,preset=args.preset,scope='真实G1激光+原生IMU与时间同步检查，不是SLAM或导航定位切换验收')
    def start(command,name,env=None):
        log=open(run/name,'w');logs.append(log)
        child=subprocess.Popen(command,env=env,start_new_session=True,stdout=log,stderr=subprocess.STDOUT)
        children.append(child);return child
    try:
        bridge=start(['/usr/bin/python3','-m','forest_nav_control.lidar_bridge'],'bridge.log')
        policy=start(['bash',str(PROJECT/'nav2_path_tracking/start_g1_policy.sh'),'--visualizer',
            'none' if args.check or args.headless else 'kit'],'simulation.log',simulation_environment())
        rclpy.init();node=Node('forest_lidar_output_check');counts=[];imus=[];synced=[];sync_status=[]
        tf=Buffer();listener=TransformListener(tf,node)
        def receive(msg):
            points=point_cloud2.read_points_numpy(msg,field_names=('x','y','z'),skip_nans=True).reshape(-1,3)
            assert msg.header.frame_id=='lidar_link'
            # 初始化/无回波帧允许为空，但不计入3帧有效点云验收；持续为空会超时。
            if len(points)==0:return
            distances=np.linalg.norm(points,axis=1)
            assert distances.min()>=.2-1e-4 and distances.max()<=20.+1e-4
            counts.append(len(points))
        sub=node.create_subscription(PointCloud2,'/lidar/points',receive,5)
        def imu_receive(msg):
            assert msg.header.frame_id=='imu_link' and msg.orientation_covariance[0]==-1.
            stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
            values=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z,
                msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
            assert np.isfinite(values).all()
            imus.append([stamp]+values)
        imu_sub=node.create_subscription(Imu,'/imu/data_raw',imu_receive,2000)
        def sync_receive(msg):
            fields={f.name:f for f in msg.fields}
            assert fields['ring'].datatype==4 and fields['time'].datatype==7
            data=point_cloud2.read_points(msg,field_names=('ring','time'))
            assert np.all(data['ring']<32) and np.all(data['time']==0.)
            synced.append(msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec)
        sync_sub=node.create_subscription(PointCloud2,'/lidar/points_synced',sync_receive,10)
        status_sub=node.create_subscription(String,'/sensors/sync_status',lambda m:sync_status.append(json.loads(m.data)),10)
        deadline=time.monotonic()+args.timeout
        print(f'正在启动G1与独立激光雷达，日志：{run}',flush=True)
        while True:
            for name,child in [('点云桥',bridge),('G1仿真',policy)]:
                if child.poll() is not None:raise RuntimeError(f'{name}提前退出，查看{run}')
            rclpy.spin_once(node,timeout_sec=.05)
            if len(counts)>=3 and len(imus)>=1000 and len(synced)>=20 and sync_status and sync_status[-1]['ready'] and not report['passed']:
                stamps=np.asarray([m[0] for m in imus],dtype=np.int64)
                assert np.all(np.diff(stamps)==5000000),'IMU缺样、重复或不是实际200Hz'
                covered=[t for t in synced if stamps[0]<=t<=stamps[-1]]
                assert len(covered)>=20 and all(t in stamps for t in covered),'点云时间未被IMU覆盖'
                transform=tf.lookup_transform('imu_link','lidar_link',rclpy.time.Time())
                assert abs(transform.transform.translation.z-.65)<1e-6
                recent=np.asarray(imus,dtype=float)[stamps>=stamps[-1]-1000000000]
                mean_acc=recent[:,4:7].mean(axis=0)
                assert 7.5<float(np.linalg.norm(mean_acc))<12.5,'静态附近IMU比力未包含正确重力'
                report.update(imu_samples=len(imus),imu_frequency_hz=200.,imu_period_ns=5000000,
                    imu_sample_gaps=0,synced_clouds=len(covered),all_synced_stamps_have_imu_sample=True,
                    mean_acceleration_last_sim_second=mean_acc.tolist(),
                    imu_topic='/imu/data_raw',synced_topic='/lidar/points_synced',
                    T_imu_lidar_translation=[transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z],
                    sync_status=sync_status[-1])
                report.update(passed=True,received_frames=len(counts),points_per_frame=counts[:3],
                    point_topic='/lidar/points',registered_topic='/lidar/points_map',
                    odometry_topic='/lidar/ground_truth_odometry',uses_camera_depth=False)
                (run/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
                print('真实激光、200Hz原生IMU、固定外参与点云时间覆盖检查通过。',flush=True)
                if args.check:break
            if not report['passed'] and time.monotonic()>deadline:raise TimeoutError(f'点云启动超时，查看{run}/simulation.log')
    except KeyboardInterrupt:pass
    except Exception as exc:
        report['error']=str(exc);raise
    finally:
        (run/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        if node and 'imus' in locals():
            np.savez_compressed(run/'sensor_measurements.npz',imu=np.asarray(imus),synced_cloud_stamps_ns=np.asarray(synced,dtype=np.int64))
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for child in reversed(children):
            try:os.killpg(child.pid,signal.SIGINT)
            except ProcessLookupError:pass
        for child in children:
            try:child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGKILL);child.wait()
        for child in children:
            try:os.killpg(child.pid,signal.SIGKILL)
            except ProcessLookupError:pass
        for log in logs:log.close()
        lock.close()

if __name__=='__main__':main()
