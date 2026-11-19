"""隔离ROS域，只调用真实Smac2D规划，不连接或控制仿真机器人。"""
import os,json,time,math,subprocess,signal,argparse
from datetime import datetime
from pathlib import Path
import yaml,rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.action import ComputePathToPose
root=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--before-config',type=Path,required=True)
parser.add_argument('--output',type=Path)
args=parser.parse_args()
if os.environ.get('ROS_DOMAIN_ID','0') in ('0','186'):raise SystemExit('请显式设置隔离的ROS_DOMAIN_ID，例如201；禁止连接正在运行的导航域')
out=args.output or root/'nav2_path_tracking/results'/('turn_clearance_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
out.mkdir(parents=True,exist_ok=False)
backup_file=args.before_config
rclpy.init();n=Node('clearance_planning_test');q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL);pub=n.create_publisher(OccupancyGrid,'/traversability_map',q)
tf=StaticTransformBroadcaster(n);t=TransformStamped();t.header.frame_id='map';t.child_frame_id='base_link';t.transform.translation.x=1.;t.transform.translation.y=5.;t.transform.rotation.w=1.;tf.sendTransform(t)
grid=OccupancyGrid();grid.header.frame_id='map';grid.info.resolution=.2;grid.info.width=60;grid.info.height=50;grid.info.origin.orientation.w=1.;grid.data=[0]*3000
for row in range(22,28):
 for col in range(25,28):grid.data[row*60+col]=100
# 地图边缘保留未知，不允许通过未知区绕行。
for c in range(60):grid.data[c]=-1;grid.data[49*60+c]=-1
pub.publish(grid);timer=n.create_timer(.1,lambda:pub.publish(grid))
def future(f,timeout=15):
 rclpy.spin_until_future_complete(n,f,timeout_sec=timeout)
 if not f.done():raise RuntimeError('规划测试接口无响应')
 return f.result()
def transition(v):
 c=n.create_client(ChangeState,'/planner_server/change_state');assert c.wait_for_service(timeout_sec=20)
 req=ChangeState.Request();req.transition.id=v;res=future(c.call_async(req));assert res.success;n.destroy_client(c)
results={}
for variant,file in [('before',backup_file),('after',root/'forest_lidar_perception/config/navigation.yaml')]:
 cfg=yaml.safe_load(file.read_text());cfg['planner_server']['ros__parameters']['use_sim_time']=False;cfg['global_costmap']['global_costmap']['ros__parameters']['use_sim_time']=False
 params=out/(variant+'.yaml');params.write_text(yaml.safe_dump(cfg))
 log=(out/(variant+'.log')).open('w');child=subprocess.Popen(['/opt/ros/humble/lib/nav2_planner/planner_server','--ros-args','--params-file',str(params)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 try:
  transition(1);transition(3)
  warmup=time.monotonic()+1.
  while time.monotonic()<warmup:rclpy.spin_once(n,timeout_sec=.05)
  client=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');assert client.wait_for_server(timeout_sec=10)
  req=ComputePathToPose.Goal();req.planner_id='GridBased';req.use_start=True
  for pose,x in [(req.start,1.),(req.goal,9.)]:pose.header.frame_id='map';pose.pose.position.x=x;pose.pose.position.y=5.;pose.pose.orientation.w=1.
  handle=future(client.send_goal_async(req));assert handle.accepted
  res=future(handle.get_result_async());poses=res.result.path.poses
  points=[(p.pose.position.x,p.pose.position.y) for p in poses]
  # 到石头所在栅格矩形边界的真实几何距离，不把单元当零面积点。
  def distance(p):return math.hypot(max(5.-p[0],0.,p[0]-5.6),max(4.4-p[1],0.,p[1]-5.6))
  # 连续线段每2cm采样，避免只检查路径节点而漏掉线段内切。
  samples=[]
  for a,b in zip(points,points[1:]):
   steps=max(1,math.ceil(math.dist(a,b)/.02))
   samples.extend((a[0]+(b[0]-a[0])*i/steps,a[1]+(b[1]-a[1])*i/steps) for i in range(steps+1))
  required=.45+math.sqrt(2)*.1
  clearance=min(map(distance,samples)) if samples else None
  result=dict(action_status=res.status,path_points=points,min_clearance_to_occupied_cell_m=clearance,local_full_rotation_radius_m=required,rotation_envelope_clear=clearance is not None and clearance>=required,path_length_m=sum(math.dist(a,b) for a,b in zip(points,points[1:])))
  assert res.status==4 and points
  results[variant]=result
  # 新配置必须保留未知屏障，不可因虚拟足迹清理而规划穿过。
  if variant=='after':
   grid.data=[-1 if 23<=c<31 else (100 if 22<=r<28 and 25<=c<28 else 0) for r in range(50) for c in range(60)]
   pub.publish(grid);end=time.monotonic()+1
   while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
   h=future(client.send_goal_async(req));rr=future(h.get_result_async());result['unknown_barrier_rejected']=rr.status!=4
  client.destroy()
 finally:
  os.killpg(child.pid,signal.SIGINT)
  try:child.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
  log.close()
 (out/'result.json').write_text(json.dumps(results,ensure_ascii=False,indent=2))
n.destroy_node();rclpy.shutdown()
assert results['after']['rotation_envelope_clear']
assert results['after']['min_clearance_to_occupied_cell_m']>results['before']['min_clearance_to_occupied_cell_m']
assert results['after']['unknown_barrier_rejected']
print(json.dumps({k:{a:b for a,b in v.items() if a!='path_points'} for k,v in results.items()},ensure_ascii=False))
