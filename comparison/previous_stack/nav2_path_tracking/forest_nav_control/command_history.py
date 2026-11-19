"""外部速度指令的逐帧历史；不依赖 ROS、Isaac 或具体张量后端。"""
from collections import deque
import math


class CommandHistory:
    """保存实际送入策略的指令，而不是把过去八帧改写成当前指令。

    环境在上一次 step 末尾生成观测，外部指令在本次推理前才到达。
    因此仅依赖环境内部的指令历史会滞后一帧；这里独立维护这一项，
    每个策略步追加一次，其余关节、动作、深度观测保持环境原样。
    初次调用用首条指令填满，与观测缓冲区初始化语义一致。
    """
    def __init__(self):
        self.frames = None

    def append(self, command, width):
        if not isinstance(width, int) or width <= 0 or width % 3:
            raise ValueError('velocity_commands 观察长度必须是正的3倍数')
        values = tuple(float(value) for value in command)
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            raise ValueError('外部速度必须包含三个有限值')
        count = width // 3
        if self.frames is None:
            self.frames = deque([values] * count, maxlen=count)
        elif self.frames.maxlen != count:
            raise ValueError('运行中速度观测历史长度发生变化')
        else:
            self.frames.append(values)
        return tuple(value for frame in self.frames for value in frame)

    def reset(self):
        """仅用于新仿真/显式重置，不因指令归零或策略切换清空历史。"""
        self.frames = None
