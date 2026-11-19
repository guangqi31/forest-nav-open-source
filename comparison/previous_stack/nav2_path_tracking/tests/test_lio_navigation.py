"""接管导航的关键边界：安装外参、消息序列化、估计控制基准和真值发布互斥。"""
from types import SimpleNamespace
import numpy as np
import rclpy
from rclpy.serialization import serialize_message,deserialize_message
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import Pose
from forest_nav_control.map_snapshot import fingerprint
from forest_nav_control.lio_navigation import control_pose,LioNavigation
from forest_nav_control.sensor_contract import rotation,quaternion_multiply
from forest_nav_control.bridge_node import SimBridge
from forest_nav_control.lio_plan_visualization import world_points


class Capture:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(msg)


def test_fingerprint_survives_ros_float32_serialization():
    msg=OccupancyGrid();msg.info.resolution=.2;msg.info.width=2;msg.info.height=1
    msg.info.origin.position.x=-16.;msg.data=[-1,100]
    encoded=deserialize_message(serialize_message(msg),OccupancyGrid)
    assert fingerprint(msg)==fingerprint(encoded)
    encoded.data[0]=0
    assert fingerprint(msg)!=fingerprint(encoded)


def test_control_root_mount_offset_not_assumed_to_be_pelvis(monkeypatch):
    import forest_nav_control.lio_navigation as module
    settings=dict(offset_torso_m=[.1,.2,.3],rotation_xyzw=[0,0,2**-.5,2**-.5])
    monkeypatch.setattr(module,'imu_settings',lambda:settings)
    root=np.array([1.,2.,3.]);q=np.array([.1,0.,0.,np.sqrt(.99)])
    imu_pos=root+rotation(q)@settings['offset_torso_m']
    imu_q=quaternion_multiply(q,settings['rotation_xyzw'])
    p,restored,yaw=control_pose(np.r_[imu_pos,imu_q])
    assert np.allclose(p,root) and np.allclose(rotation(restored),rotation(q)) and abs(yaw)<1e-10


def test_control_odom_is_estimated_and_sensor_chain_preserves_full_pose():
    obj=SimpleNamespace(last_stamp=None,last_pose=None,last_odom_wall=None,velocity=np.zeros(3),
        odom_pub=Capture(),tf=Capture())
    obj.tf.sendTransform=obj.tf.publish
    msg=Odometry();msg.header.frame_id='lio_odom';msg.child_frame_id='lio_imu';msg.header.stamp.sec=2
    msg.pose.pose.position.x=1.;msg.pose.pose.position.z=.03
    msg.pose.pose.orientation.x=np.sin(.1);msg.pose.pose.orientation.w=np.cos(.1)
    LioNavigation.on_odom(obj,msg)
    odom=obj.odom_pub.messages[-1]
    assert odom.header.frame_id=='odom' and odom.child_frame_id=='base_link'
    assert odom.pose.pose.position.z==0 and odom.pose.pose.orientation.x==0
    mount=obj.tf.messages[-1]
    assert mount.header.frame_id=='base_link' and mount.child_frame_id=='imu_link'
    assert np.isclose(mount.transform.translation.z,.03) and np.isclose(mount.transform.rotation.x,np.sin(.1))
    msg.header.stamp.nanosec=200000000;msg.pose.pose.position.x=1.1
    LioNavigation.on_odom(obj,msg)
    assert 0<obj.odom_pub.messages[-1].twist.twist.linear.x<.5


def test_lio_bridge_never_publishes_truth_odom_or_localization_tf(monkeypatch):
    monkeypatch.setenv('FOREST_NAV_LOCALIZATION','lio')
    rclpy.init();node=SimBridge()
    try:
        assert node.odom_pub is None and node.tf_pub is None and not hasattr(node,'static_pub')
        node.clock_pub=Capture()
        node.state=dict(sim_time=10.,pose=[999.,888.,1.],velocity=[7.,8.,9.])
        node._publish_state()
        assert len(node.clock_pub.messages)==1
    finally:node.destroy_node();rclpy.shutdown()


def test_coordinate_generation_change_updates_without_stopping():
    import json
    obj=SimpleNamespace(source=('a',0),generation=0,hold_until=0.,stop=Capture())
    data=dict(source=['a',0],reset_generation=1)
    LioNavigation.on_mapping(obj,SimpleNamespace(data=json.dumps(data)))
    assert obj.generation==1 and not obj.stop.messages


def test_isaac_display_reverses_map_correction_without_affecting_navigation():
    # map点(2,4)来自odom点(1,0)经90度旋转和平移(2,3)。显示再加world初始偏移。
    result=world_points([[2.,4.]],[2.,3.,np.pi/2],np.eye(3),np.array([10.,20.,0.]))
    assert np.allclose(result[0],[11.,20.,0.])


def test_live_ekf_prediction_never_hides_stopped_lio():
    import time,json,copy
    from rclpy.time import Time
    rclpy.init();node=LioNavigation()
    try:
        node.localization_pub=Capture();node.mapping_pub=Capture()
        node.get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=10.))
        grid=OccupancyGrid();grid.header.frame_id='lio_map';grid.info.width=grid.info.height=1
        grid.info.resolution=.2;grid.info.origin.orientation.w=1.;grid.data=[0]
        node.on_map(grid,0);node.on_map(grid,1)
        node.on_mapping(SimpleNamespace(data=json.dumps(dict(ready=True,source=['a',0],reset_generation=0,
            map_to_odom=[0.,0.,0.],map_fingerprints=[fingerprint(grid)]*2))))
        raw=Odometry();raw.header.frame_id='lio_odom';raw.child_frame_id='lio_imu';raw.header.stamp.sec=9
        raw.pose.pose.orientation.w=1.;node.on_odom(raw)
        filtered=copy.deepcopy(raw);filtered.header.frame_id='odom';filtered.child_frame_id='base_link';filtered.header.stamp.sec=10
        node.on_filtered(filtered);node.tick()
        assert json.loads(node.localization_pub.messages[-1].data)['ready']
        raw.header.stamp.nanosec=900000000;node.on_odom(raw);node.tick()
        assert json.loads(node.localization_pub.messages[-1].data)['ready']
    finally:node.destroy_node();rclpy.shutdown()


def test_task_trajectory_is_in_map_after_coordinate_correction(monkeypatch):
    from forest_nav_control.navigation_client import NavigationClient
    monkeypatch.setenv('FOREST_NAV_LOCALIZATION','lio')
    transform=SimpleNamespace(translation=SimpleNamespace(x=10.,y=20.),
        rotation=SimpleNamespace(z=np.sin(np.pi/4),w=np.cos(np.pi/4)))
    obj=SimpleNamespace(samples=[],tf=SimpleNamespace(lookup_transform=lambda *args:SimpleNamespace(transform=transform)))
    msg=Odometry();msg.pose.pose.position.x=2.;msg.pose.pose.position.y=1.
    msg.pose.pose.orientation.w=1.
    NavigationClient._odom(obj,msg)
    assert np.allclose(obj.samples[-1][1:4],[9.,22.,np.pi/2])


def test_localization_report_does_not_replace_velocity():
    import time,json
    now=time.monotonic();sent=[]
    class Socket:
        def recvfrom(self,*args):raise BlockingIOError
        def sendto(self,data,destination):sent.append(json.loads(data))
    obj=SimpleNamespace(socket=Socket(),use_lio=True,state_received=now,command_received=now,
        state=dict(ready=True,session='hardware',pose=[999.,888.,0.],sim_time=10.),command=(.2,0.,0.),
        localization=dict(ready=True,pose=[1.,2.,0.],stamp=10.),localization_received=now,
        get_parameter=lambda name:SimpleNamespace(value=False),sequence=0,destination=('127.0.0.1',0),
        plan_dirty=False,plan_points=[],plan_last_sent=0.,status_pub=Capture())
    SimBridge._tick(obj);assert sent[-1]['command']==[.2,0.,0.]
    assert json.loads(obj.status_pub.messages[-1].data)['pose']==[1.,2.,0.]
    obj.localization['ready']=False
    SimBridge._tick(obj);assert sent[-1]['command']==[.2,0.,0.]
