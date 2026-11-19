"""跨ROS话题核对同一地图快照；按消息序列化精度计算，避免float32分辨率差异。"""
import hashlib
import numpy as np


def fingerprint(msg):
    geometry=np.asarray([float(np.float32(msg.info.resolution)),msg.info.width,msg.info.height,
        msg.info.origin.position.x,msg.info.origin.position.y],dtype='<f8').tobytes()
    return hashlib.sha256(geometry+np.asarray(msg.data,dtype=np.int8).tobytes()).hexdigest()
