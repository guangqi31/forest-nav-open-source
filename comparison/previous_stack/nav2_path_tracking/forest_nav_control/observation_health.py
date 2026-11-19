"""仿真观测的两种时钟：仿真时间判断测量年龄，墙钟判断运行线程是否失联。

独立进度文件由每个实际仿真步写入，不依赖LiDAR是否到达采样周期。这样慢
仿真的正常帧间隔不会被误认为雷达断流，雷达停止但仿真继续时也能被发现。
"""
import json
import math
from pathlib import Path


def scan_period():
    path=Path(__file__).resolve().parents[2]/'forest_lidar_perception/config/lidar.json'
    frequency=float(json.loads(path.read_text())['frequency_hz'])
    if not math.isfinite(frequency) or frequency<=0:raise ValueError('雷达采样频率必须为正数')
    return 1./frequency


class ProgressClock:
    """可信模拟器的轻量步进心跳；新墙钟心跳不能替代仿真时钟真正推进。"""
    def __init__(self,path,timeout=None):
        self.path=Path(path)
        self.stamp=None;self.session=None;self.sent=None;self.advanced_at=None;self.epoch=0
        self.rewind_sent=None
        self.reason='等待仿真步进心跳'

    def poll(self,now):
        try:
            data=json.loads(self.path.read_text())
            stamp=float(data['stamp']);sent=float(data['sent_monotonic']);session=data['session']
            if not isinstance(session,str) or not session or not all(math.isfinite(v) for v in (stamp,sent)) or stamp<=0:
                raise ValueError('仿真步进心跳格式无效')
            # 原子文件理论上不会乱序；防止旧文件替换当前文件而伪造时间回退。
            if session!=self.session or self.stamp is None or stamp!=self.stamp:
                if session==self.session and self.stamp is not None and stamp<self.stamp:
                    self.epoch+=1;self.rewind_sent=sent
                elif session!=self.session:self.epoch=0;self.rewind_sent=None
                self.advanced_at=sent
            self.stamp,self.session,self.sent=stamp,session,sent
        except (OSError,ValueError,TypeError,KeyError):
            # 单次文件读取失败不把原先有效心跳续命，也不立即撤销它。
            pass
        if self.sent is None:self.reason='等待仿真步进心跳';return False
        self.reason='';return True
