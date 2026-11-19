"""隔离域真实Nav2 MPPI验收，不启动Isaac或G1策略。

先source /opt/ros/humble/setup.bash，再把本项目加入PYTHONPATH。
MPPI_TEST_CASE可取straight/turn/obstacle/dynamic/handoff；默认straight。
使用ROS域197、UDP15761/15762，结果保存在/tmp/forest_mppi_<场景>。
断言超时仅用于结束测试，不写入正式任务或限制真实导航时长。
"""
import os,subprocess,signal,time,json,math,pathlib
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid,Odometry,Path
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

root=pathlib.Path(__file__).resolve().parents[1];out=pathlib.Path('/tmp/forest_mppi_'+os.environ.get('MPPI_TEST_CASE','straight'));out.mkdir(exist_ok=True)
os.environ.update(ROS_DOMAIN_ID='197',ROS_LOG_DIR=str(out/'ros'),FOREST_NAV_STATE_PORT='15761',FOREST_NAV_COMMAND_PORT='15762',FOREST_NAV_MAP_MODE='offline',FOREST_NAV_LOCALIZATION='truth',FOREST_NAV_TERMINAL_CONTROL='0',FOREST_NAV_MAX_FORWARD_SPEED='1.8',FOREST_NAV_CONTROLLER='mppi',FOREST_NAV_PARAMS=str(root.parent/'forest_lidar_perception/config/navigation.yaml'))
children=[];logs=[];node=None;report={'passed':False,'backend':'nav2_humble_mppi_kinematic_endpoint'}
def start(cmd,name):
 log=(out/(name+'.log')).open('w');logs.append(log);p=subprocess.Popen(cmd,cwd=root,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);children.append(p)
class Probe(Node):
 def __init__(self):
  super().__init__('pass_through_native_probe');self.pose=None;self.obstacle=False;self.path_max_y=0.;self.samples=[];self.plan_goal=None;self.plan_revision=0;q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.maps=[self.create_publisher(OccupancyGrid,t,q) for t in ['/traversability_map','/traversability_map_local']]
  self.controller=self.create_publisher(String,'/navigation/controller_selector',q);self.checker=self.create_publisher(String,'/navigation/goal_checker_selector',q)
  self.sub1=self.create_subscription(Odometry,'/odom',self.odom,20);self.sub2=self.create_subscription(Twist,'/cmd_vel',self.command,50)
  self.plan_sub=self.create_subscription(Path,'/plan',self.plan,10)
  self.client=ActionClient(self,NavigateToPose,'/navigate_to_pose');self.timer=self.create_timer(.2,self.publish_map)
 def plan(self,m):
  self.path_max_y=max([self.path_max_y]+[abs(p.pose.position.y) for p in m.poses])
  if m.poses:self.plan_goal=m.poses[-1].pose.position.x;self.plan_revision+=1
 def odom(self,m):
  p=m.pose.pose;self.pose=(p.position.x,p.position.y,2*math.atan2(p.orientation.z,p.orientation.w))
 def command(self,m):self.samples.append([time.monotonic(),*(self.pose or (0,0,0)),m.linear.x,m.angular.z])
 def publish_map(self):
  m=OccupancyGrid();m.header.frame_id='map';m.info.resolution=.2;m.info.width=m.info.height=200;m.info.origin.position.x=m.info.origin.position.y=-10.;m.info.origin.orientation.w=1.;m.data=[0]*40000
  if self.obstacle:
   for row in range(47,53):
    for col in range(62,68):m.data[row*200+col]=100
  for p in self.maps:p.publish(m)
 def spin(self,sec):
  end=time.monotonic()+sec
  while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.02)
 def wait(self,f,sec=5):
  end=time.monotonic()+sec
  while not f.done() and time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.02)
  assert f.done(),'ROS response timeout';return f.result()
 def send(self,x,y,yaw,final):
  revision=self.plan_revision
  if not final:
   self.controller.publish(String(data='TransitPath'));self.checker.publish(String(data='transit_goal_checker'));self.spin(.1)
  g=NavigateToPose.Goal();g.pose.header.frame_id='map';g.pose.pose.position.x=float(x);g.pose.pose.position.y=float(y);g.pose.pose.orientation.z=math.sin(yaw/2);g.pose.pose.orientation.w=math.cos(yaw/2)
  h=self.wait(self.client.send_goal_async(g));assert h.accepted,'Native goal rejected'
  if final:
   deadline=time.monotonic()+4
   while time.monotonic()<deadline and not (self.plan_revision>revision and abs((self.plan_goal or 0)-x)<.15):self.spin(.02)
   assert self.plan_revision>revision and abs(self.plan_goal-x)<.15,'New final path not received'
   self.controller.publish(String(data='FollowPath'));self.checker.publish(String(data='goal_checker'))
  return h
rclpy.init()
try:
 node=Probe();start(['bash',str(root/'start_tracking.sh'),'publish_map:=false'],'navigation');start(['/usr/bin/python3','-m','forest_nav_control.kinematic_sim','--start','0','0','0'],'endpoint')
 deadline=time.monotonic()+25
 while time.monotonic()<deadline:
  node.spin(.1)
  if node.client.server_is_ready() and node.pose is not None and (out/'navigation.log').exists() and 'Managed nodes are active' in (out/'navigation.log').read_text(errors='replace'):break
 else:raise AssertionError('Nav2 startup not active')
 case=os.environ.get('MPPI_TEST_CASE','straight')
 if case=='handoff':
  # 故意给中间点一个反向yaw，途经控制必须继续向前而不掉头对齐。
  h=node.send(2.,0.,math.pi,False);first=h.get_result_async();deadline=time.monotonic()+15
  while node.pose[0]<1.0 and time.monotonic()<deadline:
   node.spin(.03)
   assert not first.done(),'Intermediate goal ended before preemption window'
  assert node.pose[0]>=1.0,'Robot did not approach intermediate point'
  before=node.pose;cut=time.monotonic();h2=node.send(4.,0.,math.pi/2,True);second=h2.get_result_async()
  deadline=time.monotonic()+25
  while not second.done() and time.monotonic()<deadline:node.spin(.03)
  assert second.done() and second.result().status==4, 'Final navigation did not succeed'
  node.spin(.2)
  around=[s for s in node.samples if 1.4<=s[1]<=2.4]
  assert around,'No handoff samples'
  assert max(abs(s[3]) for s in around)<.6,'Robot aligned with intermediate yaw instead of passing through'
  assert min(s[4] for s in around)>.01,'Native navigation inserted a stop near intermediate point'
  # 此夹具没有项目终点闭环；检查原生成功/停车命令时的朝向，不把随后惯性漂移冒充闭环验收。
  native_success_pose=node.samples[-1][1:4]
  assert abs(math.atan2(math.sin(native_success_pose[2]-math.pi/2),math.cos(native_success_pose[2]-math.pi/2)))<.03,'Final native yaw alignment lost'
  report.update(passed=True,preemption_pose=before,intermediate_required_yaw_rad=math.pi,near_intermediate_min_command_mps=min(s[4] for s in around),near_intermediate_max_abs_yaw_rad=max(abs(s[3]) for s in around),final_native_success_pose=native_success_pose,pose_after_coast=node.pose,final_action_status=second.result().status,old_action_status=first.result().status if first.done() else None)
 else:
  if case=='straight':
   target=(10.,0.,0.)
  elif case=='turn':
   target=(4.,4.,math.pi/2)
  elif case in ('obstacle','dynamic'):
   target=(7.,0.,0.)
   node.obstacle=(case=='obstacle')
   node.spin(1.)
  else:raise ValueError(case)
  h=node.send(*target,False);result=h.get_result_async();deadline=time.monotonic()+35
  while not result.done() and time.monotonic()<deadline:
   if case=='dynamic' and node.pose[0]>.5:node.obstacle=True
   node.spin(.03)
  assert result.done() and result.result().status==4, f'{case} did not succeed'
  moving=[v for v in node.samples if v[4]>.01]
  simultaneous=[v for v in moving if abs(v[5])>.1 and v[4]>.1]
  report.update(passed=True,case=case,goal=target,final_pose=node.pose,max_vx=max(v[4] for v in moving),simultaneous_turn_samples=len(simultaneous),path_max_y=node.path_max_y,planned_paths=node.plan_revision)
  if case=='straight':
   cruise=[v[4] for v in moving if 3.<v[1]<7.]
   report['cruise_mean_vx']=sum(cruise)/len(cruise)
  if case=='turn':assert simultaneous,'No simultaneous forward/turn motion'
  if case in ('obstacle','dynamic'):
   assert node.path_max_y>1.,'Planner did not detour obstacle'
   assert all(math.hypot(max(2.4-v[1],0.,v[1]-3.6),max(-.6-v[2],0.,v[2]-.6))>=.45 for v in node.samples),'Endpoint entered obstacle footprint margin'
   if case=='dynamic':assert node.plan_revision>=2,'Invalid path was not replanned'
except Exception as exc:
 report['passed']=False;report['error']=repr(exc)
finally:
 if node is not None:(out/'samples.json').write_text(json.dumps(node.samples));node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
 for p in reversed(children):
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 for p in children:
  try:p.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
 for log in logs:log.close()
 (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False))
 if not report['passed']:raise SystemExit(1)
