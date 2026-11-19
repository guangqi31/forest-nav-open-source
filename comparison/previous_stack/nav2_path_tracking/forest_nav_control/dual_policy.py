"""G1行走/站立双策略。只产生关节动作，不修改根位姿、速度或接触状态。

模型沿用Project Instinct发布权重；状态过渡与到达验收是本项目接入逻辑，
不是官方已经验证的自动停车功能。所有时序以实际仿真步长累计，不设任务总时限。
"""
from pathlib import Path
import math
import os
import numpy as np


class StopTransition:
    """零指令先收步，降速后选择站立策略；旧模式保留原有渐变切换。

    零指令永不被延长成运动指令，因此断流/取消时仍立即向行走策略送零。
    blend只作用于同一关节映射下的动作输出，并不冻结关节或假定已停稳。
    """
    def __init__(self):
        self.weight = 0.0
        self.low_speed_time = 0.0
        self.stable_time = 0.0
        self.mode = 'walking'
        # 官方onboard选择一个agent执行，不混合两个独立策略的关节输出。
        # 新闭环默认采用同样的单策略选择；旧模式/显式blend保留原接入行为。
        self.direct_switch = (os.environ.get('FOREST_NAV_TERMINAL_CONTROL') == '1'
                              and os.environ.get('FOREST_NAV_STAND_TRANSITION','direct') == 'direct')

    def update(self, command, speed, yaw_rate, tilt, dt):
        values = [*command, speed, yaw_rate, tilt, dt]
        if not all(math.isfinite(v) for v in values) or dt <= 0:
            raise ValueError('双策略状态输入必须有限，步长必须为正')
        # 终点微调可能低于旧键盘死区；不能把有效的小转向误当成站立请求。
        terminal = os.environ.get('FOREST_NAV_TERMINAL_CONTROL') == '1'
        moving = (math.hypot(*command[:2]) > (1e-4 if terminal else .01)
                  or abs(command[2]) > (1e-4 if terminal else .03))
        if moving:
            self.low_speed_time = self.stable_time = 0.0
            self.weight = 0.0 if self.direct_switch else max(0.0, self.weight - dt / .25)
            self.mode = 'walking' if self.weight == 0 else 'starting'
        else:
            # 进入站立前核对实际机身速度和姿态，不以零指令代替实际静止。
            low = speed < .12 and abs(yaw_rate) < .20 and tilt < .35
            self.low_speed_time = self.low_speed_time + dt if low else 0.0
            if self.low_speed_time >= .20:
                self.weight = 1.0 if self.direct_switch else min(1.0, self.weight + dt / .40)
            self.mode = 'standing' if self.weight >= 1.0 else 'braking'
            stable = self.mode == 'standing' and speed < .03 and abs(yaw_rate) < .08 and tilt < .20
            self.stable_time = self.stable_time + dt if stable else 0.0
        return dict(mode=self.mode, stand_weight=self.weight, settled=self.stable_time >= 1.0,
                    stable_sim_sec=self.stable_time, measured_speed_mps=speed,
                    measured_yaw_rate_radps=yaw_rate, measured_tilt_rad=tilt)


def check_contract(walk_dir, stand_dir):
    """安全读取配置标签但不实例化Python对象；不允许仅凭维度相同拼接权重。"""
    import yaml
    configs = []
    for directory in (walk_dir, stand_dir):
        with (Path(directory) / 'params/env.yaml').open() as stream:
            configs.append(yaml.load(stream, Loader=yaml.BaseLoader))
    walk, stand = configs
    if (walk['observations']['policy'] != stand['observations']['policy']
            or walk['actions'] != stand['actions']
            or walk['scene']['robot'] != stand['scene']['robot']):
        raise ValueError('行走/站立观测、动作或机器人配置不同，必须重新适配')
    names = ('base_ang_vel', 'projected_gravity', 'velocity_commands', 'joint_pos', 'joint_vel', 'actions')
    policy = stand['observations']['policy']
    if [k for k, v in policy.items() if isinstance(v, dict)] != [*names, 'depth_image']:
        raise ValueError('双策略观测排列与当前G1契约不一致')
    if any(policy[name]['history_length'] != '8' for name in names):
        raise ValueError('双策略仅支持已核对的8帧本体历史')
    return names


class DualPolicy:
    """包装原行走推理函数；站立Actor和编码器独立，不复用行走深度特征。

    站立历史在每次进入时用当前测量初始化，随后逐帧滚动；last_action来自
    环境实际收到的混合动作，不能用尚未施加的某个策略输出替代。
    """
    def __init__(self, walk_policy, walk_dir, stand_dir, segments):
        import onnxruntime as ort
        self.names = check_contract(walk_dir, stand_dir)
        self.walk_policy = walk_policy
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        directory = Path(stand_dir) / 'exported'
        encoder = ort.InferenceSession(str(directory / '0-depth_encoder.onnx'), options,
                                      providers=['CPUExecutionProvider'])
        self.actor = ort.InferenceSession(str(directory / 'actor.onnx'), options,
                                         providers=['CPUExecutionProvider'])
        if (encoder.get_inputs()[0].shape != [1, 8, 18, 32]
                or encoder.get_outputs()[0].shape != [1, 128]
                or self.actor.get_inputs()[0].shape != [1, 896]
                or self.actor.get_outputs()[0].shape != [1, 29]):
            raise ValueError('站立ONNX模型维度与已核对的G1契约不同')
        # 官方ParkourStandAgent输入的是零深度图，不是零latent；常量编码只计算一次。
        self.latent = encoder.run(None, {encoder.get_inputs()[0].name: np.zeros((1, 8, 18, 32), np.float32)})[0]
        if not np.isfinite(self.latent).all():
            raise ValueError('站立深度编码器输出无效')
        self.slices = {}
        offset = 0
        for name, shape in segments.items():
            size = int(np.prod(shape))
            if name in self.names:
                expected = 24 if name in self.names[:3] else 232
                if size != expected:
                    raise ValueError(f'实际观测{name}长度{size}不符合契约{expected}')
                self.slices[name] = slice(offset, offset + size)
            offset += size
        if tuple(self.slices) != self.names:
            raise ValueError('实际运行时观测排列与双策略契约不一致')
        self.history = None
        self.transition = StopTransition()
        self.status = dict(mode='walking', stand_weight=0., settled=False, stable_sim_sec=0.)
        self.last_action_jump = 0.0
        self.previous = None

    def __call__(self, obs, command, speed, yaw_rate, tilt, dt):
        import torch
        self.status = self.transition.update(command, speed, yaw_rate, tilt, dt)
        weight = self.status['stand_weight']
        walk = self.walk_policy(obs) if weight < 1 else None
        if weight > 0:
            source = obs.detach().cpu().numpy()
            latest = {name: source[:, section].reshape(1, 8, -1)[:, -1:, :].copy()
                      for name, section in self.slices.items()}
            latest['velocity_commands'][:] = 0.
            if self.history is None:
                self.history = {name: np.repeat(value, 8, axis=1) for name, value in latest.items()}
            else:
                for name, value in latest.items():
                    self.history[name] = np.concatenate((self.history[name][:, 1:], value), axis=1)
            proprio = np.concatenate([self.history[name].reshape(1, -1) for name in self.names], axis=1)
            actor_input = np.concatenate((proprio, self.latent), axis=1).astype(np.float32)
            result = self.actor.run(None, {self.actor.get_inputs()[0].name: actor_input})[0]
            stand = torch.from_numpy(result).to(device=obs.device, dtype=obs.dtype)
            action = stand if walk is None else (1 - weight) * walk + weight * stand
        else:
            self.history = None
            action = walk
        if action is None or action.shape != (1, 29) or not bool(torch.isfinite(action).all()):
            raise ValueError('双策略输出不是有效29维关节动作')
        self.last_action_jump = 0. if self.previous is None else float((action - self.previous).abs().max())
        self.status['max_action_step_change'] = self.last_action_jump
        self.previous = action.detach().clone()
        return action


def parking_result(target, pose, locomotion):
    """站稳与到点分开；位置来自导航定位，不使用仿真真值修正目标。

    0.25米与0.03弧度沿用当前Nav2到达容差；站稳不在容差内明确失败，
    不冒充完成，也不绕开Nav2自行发速度纠偏。
    """
    if not locomotion.get('settled'):
        return 'settling', {}
    error = math.hypot(pose[0] - target[0], pose[1] - target[1])
    yaw = abs(math.atan2(math.sin(pose[2] - target[2]), math.cos(pose[2] - target[2])))
    return ('verified' if error <= .25 and yaw <= .03 else 'failed'), dict(
        position_error_m=error, yaw_error_rad=yaw, position_tolerance_m=.25, yaw_tolerance_rad=.03)
