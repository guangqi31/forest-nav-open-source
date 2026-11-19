"""仅一次实际G1平地停车检查：在线选点，独立几何核对，再观察零指令后的运动。"""
import os,json,time,math,signal,subprocess,sys,fcntl
from pathlib import Path
from datetime import datetime
import numpy as np
ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parent
sys.path.insert(0,str(PROJECT/'nav2_path_tracking'))


def plane_metrics(points):
    matrix=np.c_[points[:,:2],np.ones(len(points))]
    coeff=np.linalg.lstsq(matrix,points[:,2],rcond=None)[0]
    residual=points[:,2]-matrix@coeff
    return dict(slope_deg=float(np.degrees(np.arctan(np.linalg.norm(coeff[:2])))),
                height_range_m=float(np.ptp(points[:,2])),
                residual_rms_m=float(np.sqrt(np.mean(residual**2))),points=len(points))


def main():
    lock=open('/tmp/forest_nav_live_navigation.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    output=ROOT/'results'/('flat_parking_'+datetime.now().strftime('%Y%m%d_%H%M%S'));output.mkdir(parents=True)
    os.environ.update(ROS_DOMAIN_ID='197',ROS_LOG_DIR='/tmp/forest_flat_parking_ros',
        FOREST_NAV_MAP_MODE='lidar',FOREST_LIDAR_ENABLED='1',FOREST_LIDAR_PUBLISH_CLOCK='0',
        FOREST_LIDAR_FRAME_FILE=str(output/'latest_lidar.npz'),FOREST_NAV_STATE_PORT='15601',FOREST_NAV_COMMAND_PORT='15602',
        FOREST_TERRAIN_SCENE=str(PROJECT/'forest_terrain_3d/generated/basic/environment.usda'),
        FOREST_NAV_MAP=str(PROJECT/'terrain_framework_baseline/results/basic/terrain_costmap.npz'),
        FOREST_NAV_SENSOR_MESH=str(PROJECT/'forest_navigation/results/basic/sensor_obstacles.npz'),
        FOREST_NAV_PARAMS=str(ROOT/'config/navigation.yaml'))
    subprocess.run(['/usr/bin/python3',str(PROJECT/'forest_navigation/preflight.py'),'--preset','basic'],check=True,stdout=subprocess.DEVNULL)
    import rclpy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from forest_nav_control.task_client import TaskClient
    from forest_nav_control.start_navigation import simulation_environment
    report=dict(passed=False,scope='真实G1、在线激光地图、平坦停车区、零速度后5个仿真秒')
    children=[];logs=[];node=None;terrain=[]
    def start(command,name,env=None):
        log=open(output/name,'w');logs.append(log)
        child=subprocess.Popen(command,env=env,cwd=PROJECT/'nav2_path_tracking',stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(child)
    try:
        start(['bash','start_tracking.sh'],'navigation.log')
        start(['bash','start_g1_policy.sh','--visualizer','none'],'simulation.log',simulation_environment())
        rclpy.init();node=TaskClient()
        sub=node.create_subscription(PointCloud2,'/terrain_map',lambda m:terrain.append(m),2)
        node.wait_ready(timeout=150)
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            node.spin_for(.2)
            if terrain and node.goal_validator.ready:break
        assert terrain and node.goal_validator.grid is not None,'没有在线地图/地形观测'
        cloud=point_cloud2.read_points_numpy(terrain[-1],field_names=('x','y','z','intensity'),skip_nans=True)
        ground=cloud[cloud[:,3]<=.025,:3]
        m=node.goal_validator.grid;grid=np.asarray(m.data).reshape(m.info.height,m.info.width)
        yy,xx=np.indices(grid.shape);xy=np.c_[m.info.origin.position.x+(xx.ravel()+.5)*m.info.resolution,
                                            m.info.origin.position.y+(yy.ravel()+.5)*m.info.resolution]
        candidates=[];radius=.8
        for x in np.arange(.6,2.21,.2):
            for y in np.arange(-1.,1.01,.2):
                near=np.linalg.norm(xy-[x,y],axis=1)<=radius
                values=grid.ravel()[near]
                coverage=float(np.mean(values>=0))
                if coverage<.70 or np.any(values>25):continue
                patch=ground[np.linalg.norm(ground[:,:2]-[x,y],axis=1)<=radius]
                if len(patch)<80:continue
                metrics=plane_metrics(patch)
                if metrics['slope_deg']>2 or metrics['residual_rms_m']>.01 or metrics['height_range_m']>.06:continue
                try:node.goal_validator.check([x,y])
                except RuntimeError:continue
                candidates.append(dict(goal=[float(x),float(y),0.],radius_m=radius,
                    online_coverage=coverage,online_plane=metrics,score=abs(x-1.2)+abs(y)*.5))
        candidates.sort(key=lambda c:c['score']);report['online_candidates']=candidates
        assert candidates,'在线观测中没有足够平坦且覆盖充分的停车区域；不发送目标'
        # 候选仅由在线结果产生。下列同源USD几何参考只用于测试核对，不供导航使用。
        with np.load(PROJECT/'terrain_framework_baseline/results/basic/terrain_costmap.npz') as f:
            elevation=f['elevation'];origin=f['origin'];resolution=float(f['resolution']);slope=f['slope'];cost=f['cost']
        row,col=np.indices(elevation.shape);reference=np.c_[origin[0]+(col.ravel()+.5)*resolution,
                                                          origin[1]+(row.ravel()+.5)*resolution,elevation.ravel()]
        chosen=None
        for candidate in candidates:
            goal=candidate['goal'];mask=np.linalg.norm(reference[:,:2]-goal[:2],axis=1)<=radius
            pts=reference[mask]
            metrics=plane_metrics(pts)
            metrics['max_slope_deg']=float(np.degrees(np.max(slope.ravel()[mask])))
            metrics['blocked_cells']=int(np.sum(cost.ravel()[mask]>=100))
            candidate['geometry_reference']=metrics
            if metrics['slope_deg']<2 and metrics['max_slope_deg']<2 and metrics['height_range_m']<.06 and metrics['blocked_cells']==0:
                chosen=candidate;break
        assert chosen,'在线候选未通过独立地形核对；不发送目标'
        report['chosen']=chosen;goal=chosen['goal']
        (output/'selection.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        print('选择平坦停车区：'+json.dumps(chosen,ensure_ascii=False),flush=True)
        action=node.wait_task(node.submit(goal),timeout=90)
        report['action']=action
        assert action['action_status']==4,'Nav2未成功到达'
        arrival=list(node.samples[-1]);report['arrival']=arrival
        # 到达后继续运行，不改策略、不手工夹住机器人，也不提前因某一瞬间速度小就判通过。
        brake_deadline=time.monotonic()+8
        while time.monotonic()<brake_deadline:
            node.spin_for(.1)
            recent=node.adapted_commands[-3:]
            if len(recent)==3 and all(max(abs(v) for v in c[1:])<1e-6 for c in recent):break
        else:raise AssertionError('到达后速度指令未归零')
        zero_pose=list(node.samples[-1]);observation_wall=time.monotonic()
        deadline=time.monotonic()+40;start=zero_pose[0]
        while time.monotonic()<deadline:
            node.spin_for(.1)
            assert all(p.poll() is None for p in children),'测试进程提前退出'
            assert node.status and node.status['ready'],'机器人反馈失效或跌倒'
            if node.samples[-1][0]-start>=5.:break
        assert node.samples[-1][0]-start>=5.,'零指令观察时间不足'
        samples=np.asarray([s for s in node.samples if s[0]>=start])
        # 指令记录使用单调墙钟，与观察开始的单调墙钟比较。
        recent=[c for c in node.adapted_commands if c[0]>=observation_wall]
        terminal=samples[samples[:,0]>=samples[-1,0]-2.]
        drift=float(np.linalg.norm(samples[-1,1:3]-samples[0,1:3]))
        terminal_drift=float(np.linalg.norm(terminal[-1,1:3]-terminal[0,1:3]))
        yaw_drift=float(abs(np.arctan2(np.sin(terminal[-1,3]-terminal[0,3]),np.cos(terminal[-1,3]-terminal[0,3]))))
        result=dict(observed_sim_seconds=float(samples[-1,0]-start),arrival_pose=arrival[1:4],final_pose=samples[-1,1:4].tolist(),
            zero_command_start_pose=zero_pose[1:4],arrival_to_final_drift_m=float(np.linalg.norm(samples[-1,1:3]-arrival[1:3])),
            total_drift_m=drift,terminal_2s_drift_m=terminal_drift,terminal_2s_yaw_drift_rad=yaw_drift,
            terminal_mean_speed_mps=float(np.mean(np.hypot(terminal[:,4],terminal[:,5]))),
            terminal_mean_abs_wz_radps=float(np.mean(np.abs(terminal[:,6]))),
            command_zero=bool(recent and all(max(abs(v) for v in c[1:])<1e-6 for c in recent)),
            final_goal_error_m=float(np.linalg.norm(samples[-1,1:3]-goal[:2])))
        result['inside_verified_flat_area']=bool(np.max(np.linalg.norm(samples[:,1:3]-goal[:2],axis=1))+.45<=radius)
        result['standing_passed']=bool(result['inside_verified_flat_area'] and result['command_zero'] and drift<=.1 and terminal_drift<=.05
            and result['terminal_mean_speed_mps']<=.03 and result['terminal_mean_abs_wz_radps']<=.08 and yaw_drift<=.1)
        report['parking']=result;report['passed']=result['standing_passed']
        print(json.dumps(result,ensure_ascii=False),flush=True)
    except Exception as exc:
        report['error']=str(exc);print('检查结束：'+str(exc),flush=True)
    finally:
        if node:
            np.savetxt(output/'trajectory.csv',np.asarray(node.samples).reshape(-1,7),delimiter=',',header='sim_time,x,y,yaw,vx,vy,wz',comments='')
            np.savetxt(output/'commands.csv',np.asarray(node.adapted_commands).reshape(-1,4),delimiter=',',header='wall_monotonic,vx,vy,wz',comments='')
            node.destroy_node()
        (output/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
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
        for log in logs:log.close()
        lock.close()
        print('记录：'+str(output),flush=True)
    return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
