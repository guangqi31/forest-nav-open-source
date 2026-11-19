"""地图健康条件与轻量诊断：测量使用仿真时间，进程心跳使用墙钟。"""
import json
import os
import time
from pathlib import Path


def health_checks(lidar_ready,lidar_received,terrain_received,last_stamp,sim_time,now,max_sim_age=.6):
    """雷达状态是墙钟心跳；地形是实际测量，按仿真时间老化。

    不再要求CMU每2个实际秒必须产生一帧，否则慢于实时的仿真仍会误停。
    仿真冻结由雷达独立步进心跳和机器人状态看门狗发现；CMU停更而时钟继续
    则由terrain_sim_fresh发现，重复消息和坏帧均不能更新last_stamp。
    """
    return dict(lidar_valid=bool(lidar_ready),
        lidar_status_present=lidar_received is not None,
        lidar_status_fresh=lidar_received is not None and 0<=now-lidar_received<1.,
        terrain_present=terrain_received is not None,
        terrain_not_future=sim_time-last_stamp>=-.1,
        terrain_sim_fresh=0<last_stamp and sim_time-last_stamp<=max_sim_age)


class DiagnosticLog:
    def __init__(self,name):
        directory=os.environ.get('FOREST_NAV_DIAGNOSTICS_DIR')
        self.path=None if not directory else Path(directory)/(name+'.jsonl')
        if self.path:self.path.parent.mkdir(parents=True,exist_ok=True)
    def write(self,event,**fields):
        if self.path:
            try:
                with self.path.open('a') as f:
                    f.write(json.dumps(dict(event=event,wall_monotonic=time.monotonic(),**fields),ensure_ascii=False)+'\n')
            except OSError:
                pass  # 诊断磁盘错误不能中断导航回调；正式地图存档有独立错误上报。
