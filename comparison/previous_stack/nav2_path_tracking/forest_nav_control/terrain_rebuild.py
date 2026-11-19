"""保留LIO连续坐标中的观测证据，地图坐标修正后从证据重建，避免移动旧栅格产生重影。

源坐标始终是lio_odom；map_to_odom表示p_map=R(yaw)@p_odom+t。
本阶段只支持平面刚体修正，不把这个接口伪装成回环或位姿图优化器。
"""
import math
import numpy as np
from .terrain_costmap import TerrainGrid


def planar_transform(value):
    """修正量是绝对的[x米,y米,yaw弧度]，不是逐条累加的增量。"""
    transform=np.asarray(value,dtype=float)
    if transform.shape!=(3,) or not np.isfinite(transform).all():
        raise ValueError('map_to_odom必须是有限的[x,y,yaw]')
    return transform


def rebuild(groups,transform,now,ttl=None,size=32.,classified=False):
    """先在候选图中完成重建，成功后调用方才可替换公开地图。

    同一源栅格保留最近一帧的全部实际测量点。旋转后按测量坐标重新落格；
    同一目标格优先最近观测时刻，同一时刻取最大代价。未测量目标格仍为未知。
    """
    grid=TerrainGrid(ttl=ttl,size=size,expand=True)
    values=[(p,stamp) for p,stamp in groups.values() if ttl is None or now-stamp<=ttl]
    if not values:return grid
    points=np.concatenate([v[0] for v in values])
    stamps=np.concatenate([np.full(len(p),stamp) for p,stamp in values])
    x,y,yaw=planar_transform(transform);c,s=math.cos(yaw),math.sin(yaw)
    xy=points[:,:2]@np.array([[c,s],[-s,c]])+np.array([x,y])
    grid.ensure_bounds(xy)
    index=np.floor((xy-grid.origin)/grid.resolution).astype(np.int64)
    flat=index[:,1]*grid.width+index[:,0]
    np.maximum.at(grid.seen.ravel(),flat,stamps)
    newest=stamps==grid.seen.ravel()[flat]
    height=points[:,3]
    costs=(height.astype(np.int8) if classified else
        np.clip(np.rint(height*100.),0,99).astype(np.int8))
    np.maximum.at(grid.cost.ravel(),flat[newest],costs[newest])
    grid.last_stamp=float(stamps.max())
    return grid


class TerrainEvidence:
    """按源坐标栅格保存最近观测，保留每个实际点的位置，不反推或插值未观测地面。"""
    def __init__(self,classified=False):
        self.classified=classified
        self.groups={};self.last_stamp=0.;self.transform=np.zeros(3)
        self.revision=0;self.generation=0
        self.global_grid=TerrainGrid(ttl=None,expand=True)
        self.local_grid=TerrainGrid(ttl=None,expand=True)

    def observe(self,points,stamp):
        if not math.isfinite(stamp) or stamp<=0:raise ValueError('观测时间无效')
        if stamp<=self.last_stamp:return False
        points=np.asarray(points,dtype=float).reshape(-1,4)
        points=points[np.isfinite(points).all(axis=1)&(points[:,3]>=0)]
        if not len(points):return False
        keys=np.floor(points[:,:2]/.2).astype(np.int64)
        unique,inverse=np.unique(keys,axis=0,return_inverse=True)
        order=np.argsort(inverse,kind='stable');cuts=np.r_[0,np.cumsum(np.bincount(inverse))]
        candidate=dict(self.groups)
        for i,key in enumerate(unique):candidate[tuple(key)]=(points[order[cuts[i]:cuts[i+1]]].copy(),stamp)
        global_grid=rebuild(candidate,self.transform,stamp,size=self.global_grid.size,classified=self.classified)
        local_grid=rebuild(candidate,self.transform,stamp,ttl=None,classified=self.classified)
        self.groups=candidate;self.global_grid=global_grid;self.local_grid=local_grid;self.last_stamp=stamp
        return True

    def correct(self,transform,revision,now):
        if type(revision) is not int or revision<=self.revision:raise ValueError('坐标修正版号必须严格递增')
        transform=planar_transform(transform)
        # 两张候选图都构建成功才提交；错误请求不能清图或改变当前坐标版本。
        global_grid=rebuild(self.groups,transform,now,classified=self.classified)
        local_grid=rebuild(self.groups,transform,now,ttl=None,classified=self.classified)
        self.global_grid,self.local_grid=global_grid,local_grid
        self.transform=transform;self.revision=revision;self.generation+=1
