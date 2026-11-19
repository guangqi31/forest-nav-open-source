"""真实上游C++进程的图构建、带ID路线输出、取消与旧取消隔离。"""
import os
import subprocess
import time
from pathlib import Path
import pytest
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2,PointField
from sensor_msgs_py import point_cloud2
from rosgraph_msgs.msg import Clock
from visibility_graph_msg.msg import ForestGoal,ForestRoute


def test_real_far_graph_route_and_cancel(tmp_path):
    executable=os.environ.get('FOREST_FAR_TEST_EXECUTABLE')
    config=os.environ.get('FOREST_FAR_TEST_CONFIG')
    if not executable:pytest.skip('set FOREST_FAR_TEST_EXECUTABLE to built upstream binary')
    log=(tmp_path/'far.log').open('w')
    process=subprocess.Popen([executable,'--ros-args','--params-file',config,
        '-r','__node:=far_planner','-r','/odom_world:=/test_far/odom','-r','/terrain_cloud:=/test_far/terrain'],stdout=log,stderr=subprocess.STDOUT)
    rclpy.init();n=Node('far_upstream_contract_test');routes=[]
    sub=n.create_subscription(ForestRoute,'/far/route',routes.append,10)
    req=n.create_publisher(ForestGoal,'/far/request',10)
    odom_pub=n.create_publisher(Odometry,'/test_far/odom',10)
    cloud_pub=n.create_publisher(PointCloud2,'/test_far/terrain',10)
    clock_pub=n.create_publisher(Clock,'/clock',10)
    fields=[PointField(name=name,offset=i*4,datatype=7,count=1) for i,name in enumerate(('x','y','z','intensity'))]
    points=[(x*.2,y*.2,0.,0.) for x in range(-30,31) for y in range(-30,31)]
    points += [(3.+x*.1,y*.1,z*.1,.5) for x in range(5) for y in range(-5,6) for z in range(1,16)]
    stamp=1.
    def pump_until(predicate,timeout=15.):
        nonlocal stamp
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            assert process.poll() is None,(tmp_path/'far.log').read_text()[-4000:]
            stamp+=.05;sec=int(stamp);nano=int((stamp-sec)*1e9)
            clock=Clock();clock.clock.sec=sec;clock.clock.nanosec=nano;clock_pub.publish(clock)
            odom=Odometry();odom.header.frame_id='lio_odom';odom.header.stamp=clock.clock
            odom.child_frame_id='far_control_root';odom.pose.pose.position.z=.75;odom.pose.pose.orientation.w=1.
            odom_pub.publish(odom)
            cloud=point_cloud2.create_cloud(odom.header,fields,points);cloud_pub.publish(cloud)
            rclpy.spin_once(n,timeout_sec=.05)
            if predicate():return
        raise AssertionError('FAR condition timeout\n'+(tmp_path/'far.log').read_text()[-5000:])
    def send(task,cancel=False):
        msg=ForestGoal();msg.header.frame_id='lio_odom';msg.task_id=task;msg.cancel=cancel;msg.goal.x=-3.;req.publish(msg)
    try:
        pump_until(lambda:any(r.ready for r in routes))
        send('first')
        pump_until(lambda:any(r.task_id=='first' and r.valid and len(r.path.poses)>=2 for r in routes))
        route=next(r for r in reversed(routes) if r.task_id=='first' and r.valid)
        assert route.path.header.frame_id=='lio_odom'
        send('first',True)
        pump_until(lambda:any(r.task_id=='first' and r.canceled for r in routes))
        send('second')
        pump_until(lambda:any(r.task_id=='second' and r.valid for r in routes))
        routes.clear();send('first',True)
        pump_until(lambda:any(r.task_id=='second' and r.valid for r in routes))
        assert not any(r.task_id=='second' and r.canceled for r in routes)
    finally:
        process.terminate()
        try:process.wait(timeout=5.)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close();n.destroy_node();rclpy.shutdown()
