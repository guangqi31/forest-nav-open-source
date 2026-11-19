"""地图保留/清除和上层状态契约检查；不启动Isaac。"""
import json,os,tempfile,time,unittest
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header,String
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
from forest_nav_control.terrain_costmap import TerrainGrid,TerrainCostmap
from forest_nav_control.mission_status import interpret_phase


def cell(grid,x,y):return tuple(np.floor((np.array([y,x])-grid.origin)/grid.resolution).astype(int))


class GridRules(unittest.TestCase):
 def test_global_retains_old_area(self):
  g=TerrainGrid(ttl=None);g.update([[0,0,0,0]],1);g.update([[3,3,0,0]],100)
  self.assertEqual(g.cost[cell(g,0,0)],0)
 def test_local_expiry_is_unknown(self):
  g=TerrainGrid(ttl=3);g.update([[0,0,.4,.4]],1);g.expire(5)
  self.assertEqual(g.cost[cell(g,0,0)],-1)
 def test_positive_ground_observation_clears_old_obstacle(self):
  g=TerrainGrid(ttl=None);g.update([[0,0,.4,.4]],1);g.update([[0,0,0,0]],2)
  self.assertEqual(g.cost[cell(g,0,0)],0)
 def test_unobserved_obstacle_is_not_cleared(self):
  g=TerrainGrid(ttl=None);g.update([[0,0,.4,.4]],1);g.update([[1,1,0,0]],2)
  self.assertEqual(g.cost[cell(g,0,0)],40)
 def test_same_frame_obstacle_overrides_ground(self):
  g=TerrainGrid(ttl=None);g.update([[0,0,0,0],[0,0,.4,.4]],1)
  self.assertEqual(g.cost[cell(g,0,0)],40)
 def test_old_message_never_clears_history(self):
  g=TerrainGrid(ttl=None);g.update([[0,0,0,0]],10);g.update([[3,3,0,0]],1)
  self.assertEqual(g.cost[cell(g,0,0)],0)
 def test_reserved_exploration_is_not_emitted(self):
  self.assertEqual(interpret_phase('idle',False,True,True),'ready')
  self.assertEqual(interpret_phase('idle',False,True,True,{'state':'rejected','reason_code':'unknown'}),'waiting_observation')
 def test_rejected_new_request_does_not_hide_active_navigation(self):
  self.assertEqual(interpret_phase('navigating',True,True,True,{'state':'rejected','reason_code':'unknown'}),'navigating')
 def test_health_fault_and_action_failure_differ(self):
  self.assertEqual(interpret_phase('failed',False,True,True),'blocked')
  self.assertEqual(interpret_phase('failed',True,False,False,latched_fault=True),'fault')


class RosMapLifecycle(unittest.TestCase):
 def test_dual_topics_expiry_reobserve_save_reset_and_session(self):
  with tempfile.TemporaryDirectory(prefix='forest_maps_') as td:
   os.environ['FOREST_NAV_MAP_ARCHIVE']=td
   rclpy.init();mapper=TerrainCostmap();observer=Node('map_lifecycle_observer');messages={'global':[],'local':[],'status':[]}
   qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
   subs=[observer.create_subscription(OccupancyGrid,topic,lambda msg,key=key:messages[key].append(msg),qos)
      for key,topic in [('global','/traversability_map'),('local','/traversability_map_local')]]
   subs.append(observer.create_subscription(String,'/navigation/mapping_status',lambda msg:messages['status'].append(json.loads(msg.data)),10))
   clock=observer.create_publisher(Clock,'/clock',10)
   def spin_until(predicate,timeout=2):
    end=time.monotonic()+timeout
    while not predicate() and time.monotonic()<end:
     rclpy.spin_once(mapper,timeout_sec=.01);rclpy.spin_once(observer,timeout_sec=.01)
    self.assertTrue(predicate())
   def set_clock(sec):
    end=time.monotonic()+.15
    while time.monotonic()<end:
     clock.publish(Clock(clock=Time(sec=sec)));rclpy.spin_once(mapper,timeout_sec=.01)
   def terrain(sec,rows):
    from sensor_msgs.msg import PointField
    fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,name in enumerate(('x','y','z','intensity'))]
    mapper.on_terrain(point_cloud2.create_cloud(Header(frame_id='map',stamp=Time(sec=sec)),fields,rows))
   try:
    set_clock(10);mapper.on_lidar(String(data=json.dumps({'ready':True,'session':'first'})))
    terrain(10,[[0.,0.,0.,0.],[1.,0.,.4,.4]])
    spin_until(lambda:messages['global'] and any(v==40 for v in messages['global'][-1].data))
    self.assertEqual(mapper.grid.cost[cell(mapper.grid,1,0)],40)
    set_clock(15)
    spin_until(lambda:messages['local'] and any(v>=0 for v in messages['local'][-1].data))
    self.assertEqual(mapper.grid.cost[cell(mapper.grid,0,0)],0)
    spin_until(lambda:messages['status'] and messages['status'][-1]['ready'])
    mapper.on_lidar(String(data=json.dumps({'ready':True,'session':'first'})));terrain(15,[[1.,0.,0.,0.]])
    self.assertEqual(mapper.grid.cost[cell(mapper.grid,1,0)],0)
    self.assertEqual(mapper.local_grid.cost[cell(mapper.grid,0,0)],0)
    save=observer.create_client(Trigger,'/mapping/save');self.assertTrue(save.wait_for_service(timeout_sec=1))
    future=save.call_async(Trigger.Request());spin_until(future.done)
    self.assertTrue(future.result().success);folder=Path(future.result().message)
    with np.load(folder/'global.npz') as f:self.assertTrue((f['cost']>=0).any())
    reset=observer.create_client(Trigger,'/mapping/reset');self.assertTrue(reset.wait_for_service(timeout_sec=1))
    future=reset.call_async(Trigger.Request());spin_until(future.done)
    self.assertTrue(future.result().success);self.assertTrue((mapper.grid.cost==-1).all())
    mapper.on_lidar(String(data=json.dumps({'ready':True,'session':'second'})))
    self.assertTrue((folder/'global.npz').exists(),'旧会话存档被删除')
   finally:
    observer.destroy_node();mapper.destroy_node();rclpy.shutdown();del os.environ['FOREST_NAV_MAP_ARCHIVE']

if __name__=='__main__':unittest.main(verbosity=2)
