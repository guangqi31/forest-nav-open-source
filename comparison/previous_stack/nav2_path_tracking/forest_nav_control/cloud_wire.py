"""仅本机的点云数据包；与速度控制协议分离，不携带任何控制命令。"""
import struct
import time
import numpy as np
HEADER = struct.Struct('<4sddI')
PORT = 15471


def encode(points, stamp):
    points = np.asarray(points, dtype='<f4').reshape(-1,3)
    if not np.isfinite(points).all():
        raise ValueError('点云包含无效数值')
    return HEADER.pack(b'FPC1', float(stamp), time.monotonic(), len(points)) + points.tobytes()


def decode(data):
    if len(data)<HEADER.size: raise ValueError('点云包过短')
    magic,stamp,sent,count=HEADER.unpack_from(data)
    if magic!=b'FPC1' or not 0<count or len(data)!=HEADER.size+count*12:
        raise ValueError('点云包格式错误')
    if not np.isfinite(stamp) or stamp<=0:
        raise ValueError('点云时间无效')
    points=np.frombuffer(data,dtype='<f4',offset=HEADER.size).reshape(-1,3)
    if not np.isfinite(points).all(): raise ValueError('点云包含无效数值')
    return stamp,points


CHUNK = struct.Struct('<4sQII')


def encode_chunks(points,stamp):
    payload=encode(points,stamp)
    frame=time.monotonic_ns();size=60000-CHUNK.size
    total=max(1,(len(payload)+size-1)//size)
    return [CHUNK.pack(b'FPC2',frame,i,total)+payload[i*size:(i+1)*size] for i in range(total)]


class CloudAssembler:
    def __init__(self):self.frames={}
    def feed(self,data):
        if data[:4]!=b'FPC2':return decode(data)
        if len(data)<CHUNK.size:raise ValueError('点云分片头不完整')
        magic,frame,index,total=CHUNK.unpack_from(data)
        if total<1 or index>=total:raise ValueError('点云分片编号无效')
        parts=self.frames.setdefault((frame,total),{})
        parts[index]=data[CHUNK.size:]
        if len(parts)!=total:return None
        payload=b''.join(parts[i] for i in range(total));del self.frames[(frame,total)]
        return decode(payload)
