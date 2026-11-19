"""ROS任务记录发布器。状态只用于显示，绝不发布速度或改变控制许可。"""
import json
import uuid
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from .task_records import TaskRecords


class TaskReporter(TaskRecords):
    def __init__(self, node, source):
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = node.create_publisher(String, '/navigation/task_records', qos)
        super().__init__(source + ':' + uuid.uuid4().hex[:8], self._publish)
        self.parking_sub=node.create_subscription(String,'/navigation/exploration_status',self._parking,10)

    def _parking(self,msg):
        # 按上层Action UUID关联停车证据，旧任务不能覆盖新任务。
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):return
        if data.get('parking_status') not in ('settling','verified','failed'):return
        for key,record in list(self.records.items()):
            if record.get('goal_id')==data.get('goal_id') and data.get('goal_id'):
                fields=dict(parking_status=data['parking_status'],parking_detail=data.get('parking_detail',{}))
                if data['parking_status']=='verified' and record['state']=='succeeded':
                    fields['reason']='导航到达且站立反馈、终点位姿检查通过'
                self.update(key,**fields)

    def _publish(self, snapshot):
        self.publisher.publish(String(data=json.dumps(snapshot, ensure_ascii=False, allow_nan=False)))
