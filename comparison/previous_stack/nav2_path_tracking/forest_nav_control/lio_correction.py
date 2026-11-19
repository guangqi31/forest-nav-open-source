"""提交一条绝对平面地图修正，等待对应请求确认；不修改机器人速度或连续LIO状态。"""
import argparse
import json
import math
import time
import uuid
import rclpy
from std_msgs.msg import String


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x',type=float,required=True,help='map→odom绝对X平移，米')
    parser.add_argument('--y',type=float,required=True,help='map→odom绝对Y平移，米')
    parser.add_argument('--yaw-deg',type=float,required=True,help='map→odom绝对偏航角，度')
    args=parser.parse_args()
    if not all(math.isfinite(v) for v in (args.x,args.y,args.yaw_deg)):parser.error('修正量必须有限')
    rclpy.init();node=rclpy.create_node('lio_map_correction_client');latest=[]
    sub=node.create_subscription(String,'/lio/mapping_status',lambda m:latest.append(json.loads(m.data)),10)
    publisher=node.create_publisher(String,'/lio/mapping/correction',10)
    request_id=uuid.uuid4().hex
    try:
        deadline=time.monotonic()+5.
        while not latest or not latest[-1].get('source'):
            rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()>deadline:raise TimeoutError('没有有效LIO地图会话，请核对ROS_DOMAIN_ID和启动状态')
        state=latest[-1];source=state['source'];revision=state['correction_revision']
        request=dict(schema_version=1,request_id=request_id,session=source[0],clock_epoch=source[1],
            base_revision=revision,revision=revision+1,frame_id='lio_map',child_frame_id='lio_odom',
            map_to_odom=[args.x,args.y,math.radians(args.yaw_deg)])
        while publisher.get_subscription_count()==0:
            rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()>deadline:raise TimeoutError('修正接收端未就绪')
        publisher.publish(String(data=json.dumps(request)))
        deadline=time.monotonic()+10.
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            ack=latest[-1].get('last_correction')
            if ack and ack.get('request_id')==request_id:
                if not ack['accepted']:raise RuntimeError(ack['reason'])
                print('坐标修正已提交，版本'+str(ack['revision'])+'；地图已按原始观测重建。',flush=True)
                return
        raise TimeoutError('未收到本次修正确认，请先查看mapping_status，勿直接重复提交')
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
