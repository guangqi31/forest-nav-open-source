"""统一就绪探针：读取真实ROS状态，既不伪造定位，也不提交目标。"""
import json
import os
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


class Readiness(Node):
    def __init__(self):
        super().__init__('forest_startup_readiness')
        self.bridge, self.guard = {}, {}
        self.received = {}
        self.action = ActionClient(self, NavigateToPose, '/navigation/navigate_to_goal' if os.environ.get('FOREST_NAV_MAP_MODE')=='lidar' else '/navigate_to_pose')
        self.lifecycle_clients = {name:self.create_client(GetState, '/' + name + '/get_state')
                        for name in (('controller_server',) if os.environ.get('FOREST_NAV_PLANNER')=='far' else ('planner_server', 'controller_server', 'bt_navigator'))}
        self.active, self.pending = {}, {}
        self.far = {}; self.far_received = 0.
        if os.environ.get('FOREST_NAV_PLANNER')=='far':
            self.far_sub=self.create_subscription(String,'/navigation/far_status',self.on_far,10)
        self.pub = self.create_publisher(String, '/navigation/startup_status', 10)
        self.subs = [self.create_subscription(String, '/navigation/' + topic,
            lambda msg, key=key:self.receive(key, msg), 10)
            for key,topic in [('bridge','bridge_status'), ('guard','task_status')]]

    def on_far(self,msg):
        try:self.far=json.loads(msg.data);self.far_received=time.monotonic()
        except (ValueError,TypeError):pass

    def receive(self, key, msg):
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict):
                setattr(self, key, data)
                self.received[key] = time.monotonic()
        except (ValueError, TypeError):
            pass

    def report(self):
        now = time.monotonic()
        for name, client in self.lifecycle_clients.items():
            if name in self.pending and self.pending[name].done():
                try:
                    self.active[name] = (self.pending[name].result().current_state.id == 3, now)
                except Exception:
                    self.active[name] = (False, now)
                del self.pending[name]
            if name not in self.pending and client.service_is_ready() and now-self.active.get(name,(False,0))[1] > 1:
                self.pending[name] = client.call_async(GetState.Request())
        checks = dict(robot_state_and_policy=bool(self.bridge.get('pose') is not None),
            map_tf_and_control=bool(self.guard.get('map_valid') and self.guard.get('tf_available')),
            nav2_lifecycle=all(self.active.get(name,(False,0))[0] for name in self.lifecycle_clients),
            navigate_action=self.action.server_is_ready())
        if os.environ.get('FOREST_NAV_PLANNER')=='far':checks['far_planner']=bool(self.far.get('ready'))
        reasons = {'far_planner':'等待FAR图与路径接口就绪', 'robot_state_and_policy':'等待G1策略与有效机器人反馈',
            'map_tf_and_control':self.guard.get('reason') or '等待地图和TF',
            'nav2_lifecycle':'等待Nav2规划、控制和行为树激活', 'navigate_action':'等待坐标导航Action'}
        return dict(ready=all(checks.values()), checks=checks,
                    reason='；'.join(reasons[k] for k,v in checks.items() if not v),
                    updated_wall_time=time.time())


def main():
    rclpy.init()
    node = Readiness()
    target = Path(os.environ['FOREST_NAV_READINESS_FILE'])
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
            report = node.report()
            temporary = target.with_suffix('.tmp')
            temporary.write_text(json.dumps(report, ensure_ascii=False))
            temporary.replace(target)
            node.pub.publish(String(data=json.dumps(report, ensure_ascii=False)))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
