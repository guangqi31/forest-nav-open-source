"""终点位姿闭环的纯控制逻辑；不依赖 ROS、Isaac 或仿真真值。

控制器只输出机体速度，遵守现有策略只前进、只转向的能力边界。
配置是待实测标定的初始值；验收容差沿用原系统，不因本模块接入放宽。
"""
from dataclasses import dataclass, fields
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class TerminalConfig:
    position_tolerance_m: float = .25
    yaw_tolerance_rad: float = .03
    entry_fraction: float = .5
    reacquire_fraction: float = .8
    stable_sim_sec: float = 1.
    capture_radius_m: float = 1.
    handoff_radius_m: float = .5
    max_forward_mps: float = 1.8
    max_yaw_radps: float = 1.0
    position_gain: float = .6
    bearing_gain: float = 1.8
    yaw_gain: float = 1.2
    velocity_damping: float = .25
    yaw_damping: float = .15
    forward_integral_gain: float = .12
    forward_integral_limit: float = .08
    yaw_integral_gain: float = .3
    yaw_integral_limit: float = .08
    stop_learning_rate: float = .5
    max_stop_translation_m: float = .3
    max_stop_yaw_rad: float = .5
    collision_horizon_sec: float = 1.
    collision_step_sec: float = .1

    def __post_init__(self):
        if any(not math.isfinite(getattr(self, f.name)) or getattr(self, f.name) <= 0 for f in fields(self)):
            raise ValueError('终点参数必须是有限正数')
        if not 0 < self.entry_fraction < self.reacquire_fraction <= 1:
            raise ValueError('进入阈值必须小于重纠偏阈值，重纠偏不得超出验收容差')
        if self.capture_radius_m <= self.position_tolerance_m:
            raise ValueError('局部接管范围必须大于到达容差')
        if not self.position_tolerance_m < self.handoff_radius_m < self.capture_radius_m:
            raise ValueError('接管阈值必须在验收容差与局部控制范围之间')
        if self.stop_learning_rate > 1 or self.max_stop_translation_m >= self.capture_radius_m:
            raise ValueError('制动补偿学习率或范围无效')


class TerminalController:
    def __init__(self, target, config=None):
        self.cfg = config or TerminalConfig()
        self.target = tuple(float(v) for v in target)
        if len(self.target) != 3 or not all(math.isfinite(v) for v in self.target):
            raise ValueError('终点必须是有限的 x、y、yaw')
        self.phase = 'correcting'
        self.stable_time = 0.
        self.ever_verified = False
        self.detail = {}
        self.forward_integral = 0.
        self.yaw_integral = 0.
        self.last_yaw_error = None
        self.stop_offset = [0., 0., 0.]
        self.brake_pose = None
        self.position_acquired = False

    def step(self, pose, velocity, locomotion, dt):
        """只对新定位时间累计稳定时长；重复回调不会冒充持续站稳。

        先修位置、再对齐最终朝向。收步和保持使用更大的退出阈值，避免定位
        噪声触发反复起步。任何阶段偏差明显扩大都返回位置/朝向纠偏。
        """
        if len(pose) != 3 or len(velocity) != 3 or not all(math.isfinite(v) for v in (*pose, *velocity, dt)) or dt < 0:
            raise ValueError('反馈必须是有限的位姿、速度与非负步长')
        cfg = self.cfg
        dx, dy = self.target[0]-pose[0], self.target[1]-pose[1]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        ex, ey = c*dx+s*dy, -s*dx+c*dy
        distance = math.hypot(dx, dy)
        yaw_error = wrap(self.target[2]-pose[2])
        # 接管后持续纠偏；转向或收步导致距离增大时，不以固定半径终止任务。
        self.detail = dict(position_error_m=distance, yaw_error_rad=abs(yaw_error),
                           forward_error_m=ex, lateral_error_m=ey,
                           position_tolerance_m=cfg.position_tolerance_m,
                           yaw_tolerance_rad=cfg.yaw_tolerance_rad)
        if self.phase == 'settling' and locomotion.get('settled') is True and self.brake_pose is not None:
            # 每次真正站稳后，只测一次收步过程的位移。补偿只是下次收步的
            # 参考点前馈，绝不改写用户目标、定位反馈或最终验收标准。
            observed = [pose[0]-self.brake_pose[0], pose[1]-self.brake_pose[1], wrap(pose[2]-self.brake_pose[2])]
            scale = min(1., cfg.max_stop_translation_m/max(math.hypot(*observed[:2]), 1e-9))
            observed[0] *= scale; observed[1] *= scale
            observed[2] = max(-cfg.max_stop_yaw_rad, min(cfg.max_stop_yaw_rad, observed[2]))
            self.stop_offset = [(1-cfg.stop_learning_rate)*old+cfg.stop_learning_rate*new
                                for old,new in zip(self.stop_offset,observed)]
            self.brake_pose = None
        # 收步过程中保持零指令，等实际站立反馈再评估是否重起步。
        # 不能每次脚还没落稳、朝向刚偏一点，就把站立过渡打断。
        if self.phase == 'holding' or (self.phase == 'settling' and locomotion.get('settled') is True):
            if (distance > cfg.reacquire_fraction*cfg.position_tolerance_m
                    or abs(yaw_error) > cfg.reacquire_fraction*cfg.yaw_tolerance_rad):
                self.phase = 'correcting'
                self.stable_time = 0.
                self.forward_integral = self.yaw_integral = 0.
                self.position_acquired = False
        command = (0., 0., 0.)
        if self.phase == 'correcting':
            self.stable_time = 0.
            aim_dx, aim_dy = dx-self.stop_offset[0], dy-self.stop_offset[1]
            ex, ey = c*aim_dx+s*aim_dy, -s*aim_dx+c*aim_dy
            alignment_error = wrap(yaw_error-self.stop_offset[2])
            predicted_stop_error = math.hypot(aim_dx,aim_dy)
            # 位置一旦进入内圈就锁存，先完成最终朝向。轻微越过目标不能立刻
            # 转180度追点；只有预计停稳会越出外圈时才重新接近位置。
            if predicted_stop_error <= cfg.entry_fraction*cfg.position_tolerance_m:
                self.position_acquired = True
            elif predicted_stop_error > cfg.reacquire_fraction*cfg.position_tolerance_m:
                self.position_acquired = False
            if not self.position_acquired:
                self.yaw_integral = 0.; self.last_yaw_error = None
                # 不假定策略能侧移/后退；目标在身后时先原地转向。
                bearing = math.atan2(ey, ex)
                wz = cfg.bearing_gain*bearing-cfg.yaw_damping*velocity[2]
                vx = 0.
                # 使用目标在机体前向的投影连续计算速度，取消20度停走门槛。
                # 目标在身后时前向误差非正，保持只转向、不倒退的接口约定。
                if ex > 0.:
                    # 实测现有策略的小前进指令存在低速死区。只在朝向可前进且
                    # 输出未饱和时累计有限补偿，不把未知的动作能力直接放开。
                    raw = cfg.position_gain*ex+self.forward_integral-cfg.velocity_damping*velocity[0]
                    if raw < cfg.max_forward_mps:
                        self.forward_integral = min(cfg.forward_integral_limit,
                            self.forward_integral+cfg.forward_integral_gain*ex*min(dt,.1))
                    vx = max(0., cfg.position_gain*ex+self.forward_integral-cfg.velocity_damping*velocity[0])
                else:
                    self.forward_integral = 0.
                command = (min(cfg.max_forward_mps, vx), 0., max(-cfg.max_yaw_radps, min(cfg.max_yaw_radps, wz)))
            elif abs(alignment_error) > cfg.entry_fraction*cfg.yaw_tolerance_rad:
                self.forward_integral = 0.
                # 误差换向时清掉旧补偿；饱和时仅允许使输出退出饱和的积分。
                if self.last_yaw_error is not None and self.last_yaw_error*alignment_error < 0:
                    self.yaw_integral = 0.
                self.last_yaw_error = alignment_error
                raw = cfg.yaw_gain*alignment_error+self.yaw_integral-cfg.yaw_damping*velocity[2]
                if abs(raw) < cfg.max_yaw_radps or raw*alignment_error < 0:
                    self.yaw_integral = max(-cfg.yaw_integral_limit,min(cfg.yaw_integral_limit,
                        self.yaw_integral+cfg.yaw_integral_gain*alignment_error*min(dt,.1)))
                wz = cfg.yaw_gain*alignment_error+self.yaw_integral-cfg.yaw_damping*velocity[2]
                command = (0., 0., max(-cfg.max_yaw_radps, min(cfg.max_yaw_radps, wz)))
            else:
                self.phase = 'settling'
                self.brake_pose = tuple(pose)
                self.forward_integral = self.yaw_integral = 0.
                self.last_yaw_error = None
        # 曾经通过验收不代表当前仍站稳；失稳先保持零指令重新收步，
        # 位置/朝向超差仍由上面的滞回逻辑触发纠偏。
        if self.phase == 'holding' and (locomotion.get('settled') is not True
                or math.hypot(*velocity[:2]) > .03 or abs(velocity[2]) > .08):
            self.phase = 'settling'
            self.stable_time = 0.
        if self.phase == 'settling':
            stable = (locomotion.get('settled') is True
                      and distance <= cfg.position_tolerance_m
                      and abs(yaw_error) <= cfg.yaw_tolerance_rad
                      and math.hypot(*velocity[:2]) <= .03 and abs(velocity[2]) <= .08)
            self.stable_time = self.stable_time + min(dt, .1) if stable else 0.
            if self.stable_time >= cfg.stable_sim_sec:
                self.phase = 'holding'
                self.ever_verified = True
        self.detail['stable_sim_sec'] = self.stable_time
        self.detail['forward_integral_mps'] = self.forward_integral
        self.detail['yaw_integral_radps'] = self.yaw_integral
        self.detail['correction_stage'] = 'heading' if self.position_acquired else 'position'
        self.detail['stop_offset_world_m_rad'] = list(self.stop_offset)
        self.detail['braking_reference'] = [self.target[0]-self.stop_offset[0],
                                           self.target[1]-self.stop_offset[1],wrap(self.target[2]-self.stop_offset[2])]
        return command

    @property
    def verified(self):
        return self.phase == 'holding'


def trajectory_clear(grid, pose, command, config, *, pose_frame='map'):
    """对局部膨胀代价图前向投影，不重复膨胀圆形机器人足迹。

    当前 Nav2 局部图已经使用机器人半径/膨胀层。沿实际输出的圆弧逐步检查
    中心格，未知、地图外和内切/致命代价都禁止运动；旋转也检查当前位置。
    这是局部纠偏的保守检查，不能代替机器人完整三维身体碰撞验证。
    """
    if grid is None or grid.header.frame_id != pose_frame:
        return False
    info = grid.info
    if info.resolution <= 0 or len(grid.data) != info.width*info.height:
        return False
    x, y, yaw = pose
    vx, _, wz = command
    step = min(config.collision_step_sec, info.resolution/max(abs(vx), .001)/2)
    count = max(1, math.ceil(config.collision_horizon_sec/step))
    dt = config.collision_horizon_sec/count
    for i in range(count+1):
        col = math.floor((x-info.origin.position.x)/info.resolution)
        row = math.floor((y-info.origin.position.y)/info.resolution)
        if not (0 <= col < info.width and 0 <= row < info.height):
            return False
        cost = grid.data[row*info.width+col]
        if cost < 0 or cost >= 99:
            return False
        x += vx*math.cos(yaw+wz*dt/2)*dt
        y += vx*math.sin(yaw+wz*dt/2)*dt
        yaw += wz*dt
    return True


