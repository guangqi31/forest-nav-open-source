"""项目坐标入口的终点准入：只读取同一份标准二维代价话题。

可经过不等于可停稳。已有G1在代价45、约13.6度下坡终点零指令后滑移，
当前森林入口按用户设置接收代价不超过40的终点。40是当前配置值，
可由FOREST_NAV_MAX_GOAL_COST覆盖，不把它当作已标定的策略能力。
只作用于项目面板/客户端，不改变Nav2沿途代价，也不替代完整身体碰撞检查。
"""
import math
import os
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy


class GoalRejected(RuntimeError):
    def __init__(self,reason,message):
        super().__init__(message)
        self.reason=reason


class GoalMapValidator:
    def __init__(self,node):
        self.enabled=bool(os.environ.get('FOREST_TERRAIN_SCENE') or os.environ.get('FOREST_NAV_MAX_GOAL_COST'))
        self.max_cost=int(os.environ.get('FOREST_NAV_MAX_GOAL_COST','40'))
        if not 0<=self.max_cost<100:raise ValueError('终点代价上限必须在0..99之间')
        self.grid=None
        self.subscription=None
        if self.enabled:
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
            self.subscription=node.create_subscription(OccupancyGrid,'/traversability_map',self.on_map,qos)

    @property
    def ready(self):return not self.enabled or self.grid is not None

    def on_map(self,msg):
        info=msg.info;q=info.origin.orientation
        valid=(msg.header.frame_id=='map' and info.width>0 and info.height>0
               and len(msg.data)==info.width*info.height
               and math.isfinite(info.resolution) and info.resolution>0
               and all(math.isfinite(v) for v in (info.origin.position.x,info.origin.position.y,q.x,q.y,q.z,q.w))
               and abs(q.x)+abs(q.y)+abs(q.z)<1e-6 and abs(abs(q.w)-1)<1e-6)
        self.grid=msg if valid else None

    def check(self,goal,*,allow_unobserved=False):
        """检查终点；在线探索入口可接纳尚未观测的意图，不能据此直接驱动。

        范围外目标没有对应栅格，返回cost=None并标明outside；不扩图、不填自由。
        只有上层探索管理器可使用此模式。它仍须在每段运动前用已知图和
        Nav2验证观察点，最终目标被真实观测后也必须重新通过停稳代价检查。
        默认严格检查保持离线入口及直接终点准入的原行为。
        """
        if not self.enabled:return None
        if self.grid is None:raise GoalRejected('map_not_ready','正在等待有效的代价地图，目标尚未发送。')
        x,y=map(float,goal[:2])
        if not math.isfinite(x) or not math.isfinite(y):raise GoalRejected('nonfinite','目标坐标必须是有限数字。')
        info=self.grid.info
        col=math.floor((x-info.origin.position.x)/info.resolution)
        row=math.floor((y-info.origin.position.y)/info.resolution)
        if not (0<=col<info.width and 0<=row<info.height):
            if allow_unobserved:return dict(unknown=True,outside=True,cost=None,frame_id='map')
            raise GoalRejected('outside','目标超出当前地图范围，目标尚未发送。')
        cost=int(self.grid.data[row*info.width+col])
        if cost<0:
            if allow_unobserved:return dict(unknown=True,outside=False,cost=cost,frame_id='map')
            raise GoalRejected('unknown','目标位于未知区域，目标尚未发送。')
        if cost>=100:
            error=GoalRejected('occupied','目标位于禁行区域，目标尚未发送。')
            error.cost=cost;error.max_stopping_cost=self.max_cost
            raise error
        if cost>self.max_cost:
            error=GoalRejected('stopping_cost',f'目标位置代价{cost}超过当前停稳上限{self.max_cost}，尚未验证可可靠停稳，请选择其他目标。')
            error.cost=cost;error.max_stopping_cost=self.max_cost
            raise error
        return dict(unknown=False,outside=False,cost=cost,max_stopping_cost=self.max_cost,row=row,col=col,frame_id='map')
