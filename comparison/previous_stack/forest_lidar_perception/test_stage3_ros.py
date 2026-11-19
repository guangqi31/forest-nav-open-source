"""真实ROS节点/原生Nav2静态层接口检查；受控测量注入，不是物理大场景行走。

在独立ROS域验证地图尺寸变化、整图更新、修正后的旧障碍清除和代次反馈。
不启动Isaac、不训练、不向实际机器人发送速度。
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2,PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header,String
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_msgs.srv import GetCostmap
from forest_nav_control.lio_costmap import LioCostmap

ROOT=Path(__file__).resolve().parent


def main():
    output=ROOT/'results/mapping_stage3_ros';output.mkdir(parents=True,exist_ok=True)
    os.environ['FOREST_LIO_OUTPUT']=str(output)
    params=output/'nav2.yaml'
    params.write_text('''/planner_server:
  ros__parameters:
    use_sim_time: false
    planner_plugins: [GridBased]
    GridBased:
      plugin: nav2_smac_planner/SmacPlanner2D
/global_costmap/global_costmap:
  ros__parameters:
    use_sim_time: false
    global_frame: lio_map
    robot_base_frame: stage3_base
    resolution: 0.2
    update_frequency: 10.0
    publish_frequency: 10.0
    track_unknown_space: true
    rolling_window: false
    plugins: [static_layer]
    static_layer:
      plugin: nav2_costmap_2d::StaticLayer
      map_topic: /lio/traversability_map
      map_subscribe_transient_local: true
      subscribe_to_updates: false
      footprint_clearing_enabled: false
''')
    log=open(output/'nav2.log','w')
    child=subprocess.Popen(['/opt/ros/humble/lib/nav2_planner/planner_server','--ros-args','--params-file',str(params)],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    report=dict(passed=False,scope='受控输入的真实ROS地图节点+原生Nav2静态层接口，不是物理行走')
    rclpy.init();mapper=LioCostmap();observer=Node('stage3_ros_check')
    clock=observer.create_publisher(Clock,'/clock',10)
    status=observer.create_publisher(String,'/lio/status',10)
    terrain=observer.create_publisher(PointCloud2,'/lio/terrain_map',5)
    correction=observer.create_publisher(String,'/lio/mapping/correction',5)
    states=[];maps=[]
    subs=[observer.create_subscription(String,'/lio/mapping_status',lambda m:states.append(json.loads(m.data)),10)]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    from nav_msgs.msg import OccupancyGrid
    subs.append(observer.create_subscription(OccupancyGrid,'/lio/traversability_map',maps.append,qos))
    tf=StaticTransformBroadcaster(observer);transform=TransformStamped(child_frame_id='stage3_base')
    transform.header.frame_id='lio_map';transform.transform.rotation.w=1.;tf.sendTransform(transform)
    sim=[10]
    def spin(predicate=lambda:False,timeout=3.,required=True):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            clock.publish(Clock(clock=Time(sec=sim[0])))
            status.publish(String(data=json.dumps(dict(ready=True,source=['stage3-interface-test',0]))))
            rclpy.spin_once(mapper,timeout_sec=.005);rclpy.spin_once(observer,timeout_sec=.005)
            if predicate():return
            if child.poll() is not None:raise RuntimeError('Nav2提前退出')
        if required:raise TimeoutError('接口条件未及时满足')
    def service(kind):
        found=[name for name,types in observer.get_service_names_and_types() if kind in types]
        if kind=='lifecycle_msgs/srv/ChangeState':return '/planner_server/change_state' if '/planner_server/change_state' in found else None
        return found[0] if found else None
    def call(client,request):
        future=client.call_async(request);spin(future.done);return future.result()
    fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z','intensity'))]
    def cloud(rows):
        return point_cloud2.create_cloud(Header(frame_id='lio_odom',stamp=Time(sec=sim[0])),fields,rows)
    def cost_at(costmap,x,y):
        m=costmap.metadata
        col,row=np.floor((np.array([x-m.origin.position.x,y-m.origin.position.y]))/m.resolution).astype(int)
        return costmap.data[row*m.size_x+col]
    try:
        spin(lambda:mapper.source is not None)
        terrain.publish(cloud([[1.1,.1,0.,.4]]));spin(lambda:mapper.updates==1)
        spin(lambda:service('lifecycle_msgs/srv/ChangeState') is not None)
        client=observer.create_client(ChangeState,service('lifecycle_msgs/srv/ChangeState'))
        assert call(client,ChangeState.Request(transition=Transition(id=1))).success
        assert call(client,ChangeState.Request(transition=Transition(id=3))).success
        spin(lambda:service('nav2_msgs/srv/GetCostmap') is not None)
        getter=observer.create_client(GetCostmap,service('nav2_msgs/srv/GetCostmap'))
        def get():return call(getter,GetCostmap.Request()).map
        initial=get();assert initial.metadata.size_x==160 and cost_at(initial,1.1,.1)==254
        sim[0]=11;spin(lambda:mapper.get_clock().now().nanoseconds>=11000000000)
        terrain.publish(cloud([[20.1,-17.1,0.,0.]]));spin(lambda:mapper.updates==2)
        latest=[None]
        def expanded():
            latest[0]=get();return latest[0].metadata.size_x==240
        spin(expanded,timeout=5.)
        assert cost_at(latest[0],1.1,.1)==254 and cost_at(latest[0],20.1,-17.1)==0
        assert cost_at(latest[0],-20,20)==255
        report['expanded_size_cells']=latest[0].metadata.size_x
        report['historical_obstacle_preserved']=True;report['new_border_unknown']=True
        request=dict(schema_version=1,session='stage3-interface-test',clock_epoch=0,revision=1,
            frame_id='lio_map',child_frame_id='lio_odom',map_to_odom=[4.,0.,0.])
        correction.publish(String(data=json.dumps(request)))
        spin(lambda:mapper.evidence.revision==1)
        def moved():
            latest[0]=get()
            return cost_at(latest[0],1.1,.1)==255 and cost_at(latest[0],5.1,.1)==254
        spin(moved,timeout=5.)
        report['correction_cleared_old_obstacle']=True;report['correction_revision']=1
        assert mapper.evidence.generation==1
        sim[0]=12;spin(lambda:mapper.get_clock().now().nanoseconds>=12000000000)
        terrain.publish(cloud([[1.1,.1,0.,0.]]));spin(lambda:mapper.updates==3)
        spin(lambda:cost_at(get(),5.1,.1)==0)
        report['new_observation_uses_same_correction']=True
        # 使用交付给用户的客户端提交第二次绝对修正，并等待其独立request_id确认。
        with (output/'correction_client.log').open('w') as cli_log:
            cli=subprocess.Popen(['bash',str(ROOT/'correct_lio_map.sh'),'--x','0','--y','0','--yaw-deg','0'],
                stdout=cli_log,stderr=subprocess.STDOUT)
            try:spin(lambda:cli.poll() is not None,timeout=12.)
            finally:
                if cli.poll() is None:cli.terminate();cli.wait(timeout=3.)
            assert cli.returncode==0
        assert mapper.evidence.revision==2 and mapper.evidence.generation==2
        report['correction_client_confirmed_revision']=2
        mapper.save_maps()
        # 原生生命周期节点先停用、清理插件，再退出进程，避免直接中断活动插件的析构竞态。
        assert call(client,ChangeState.Request(transition=Transition(id=4))).success
        assert call(client,ChangeState.Request(transition=Transition(id=2))).success
        report['nav2_lifecycle_cleanup']=True;report['passed']=True
    except Exception as exc:report['error']=repr(exc)
    finally:
        observer.destroy_node();mapper.destroy_node();rclpy.shutdown()
        try:os.killpg(child.pid,signal.SIGINT);child.wait(timeout=5.)
        except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        report['native_process_returncode']=child.returncode
        report['passed']=report['passed'] and child.returncode==0
        log.close();(output/'verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
