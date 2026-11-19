"""任务反馈数据模型：不依赖ROS，任务结果与物理停车结论分别记录。"""
import copy
import time
import uuid


class TaskRecords:
    """保存本客户端最近任务；旧任务迟到反馈不能覆盖新任务。"""
    def __init__(self, source, publish=lambda data: None):
        self.source, self.publish = source, publish
        self.current_id = None
        self.records = {}

    @property
    def current(self):
        return self.records.get(self.current_id)

    def begin(self, goal):
        task_id = uuid.uuid4().hex
        self.current_id = task_id
        self.records[task_id] = dict(task_id=task_id, source=self.source,
            goal=dict(frame_id='map', x=goal[0], y=goal[1], yaw_rad=goal[2]),
            state='submitting', reason='', reason_code='', goal_id=None, pose=None,
            distance_remaining_m=None, action_status=None, error_code=None,
            error_msg='', parking_status='unverified', replaced_by=None,
            created_wall_time=time.time(), updated_wall_time=time.time())
        # 限制内存使用；仍在执行的旧任务不能因历史条目过多被丢弃。
        for key in list(self.records):
            if len(self.records) <= 50:
                break
            if key != self.current_id and self.records[key]['state'] in (
                    'succeeded', 'canceled', 'failed', 'rejected'):
                del self.records[key]
        self.update(task_id)
        return task_id

    def update(self, task_id, **fields):
        if task_id not in self.records:
            return
        record = self.records[task_id]
        # 已结束任务可能收到队列中迟到的执行反馈，禁止退回执行中；取消请求也不被反馈冲掉。
        if fields.get('state') == 'navigating' and record['state'] in (
                'succeeded', 'canceled', 'failed', 'rejected', 'canceling'):
            return
        record.update(fields, updated_wall_time=time.time())
        self.publish(self.snapshot())

    def accepted(self, task_id, goal_id):
        # 只有新目标真的被Nav2接受，才登记旧任务被替换；新目标被拒绝时旧任务继续。
        for key, record in self.records.items():
            if task_id == self.current_id and key != task_id and record['state'] in ('accepted', 'navigating', 'canceling'):
                record['replaced_by'] = task_id
        self.update(task_id, state='accepted', goal_id=goal_id)

    def result(self, task_id, status, error_code=None, error_msg=''):
        state = {4:'succeeded', 5:'canceled', 6:'failed'}.get(status, 'failed')
        verified=self.records.get(task_id,{}).get('parking_status')=='verified'
        reason = ('导航到达且站立反馈、终点位姿检查通过' if status==4 and verified else error_msg) or {4:'Nav2返回到达；物理停稳尚未验证',
            5:'Nav2确认取消；物理停稳尚未验证',
            6:'Nav2执行失败；请结合导航日志查看原因'}.get(status, '未知Action结果')
        self.update(task_id, state=state, action_status=status, error_code=error_code,
                    error_msg=error_msg, reason=reason, distance_remaining_m=None)

    def snapshot(self):
        return copy.deepcopy(dict(schema_version=1, source=self.source,
            current_task=self.current,
            active_tasks=[r for r in self.records.values() if r['state'] in (
                'accepted', 'navigating', 'canceling') and not r['replaced_by']],
            recent_tasks=list(self.records.values())))
