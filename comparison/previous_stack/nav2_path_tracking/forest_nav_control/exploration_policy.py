"""目标引导探索的纯数据决策：上游找边界，本模块只挑选已知区的观察位置。

不实现全局寻路、不把未知格当空地。候选经过真实 Nav2 膨胀图筛选后，
还必须由管理节点调用 ComputePathToPose 验证，最终才可下发 NavigateToPose。
"""
import math
import os
import numpy as np


def grid_array(grid):
    return np.asarray(grid.data,dtype=np.int16).reshape(grid.info.height,grid.info.width)


def observation_candidates(raw, inflated, frontiers, pose, target, excluded=(), limit=8,
                            with_metadata=False):
    """输出若干分散候选，优先接近最终目标；失败/已观察位置整次任务不再选取。

    inflated 必须是 Nav2 发布的 OccupancyGrid（-1、0..100），不是原始0..255。
    99/100分别表示足迹内切区和致命障碍，二者及未知全部排除。
    最终路径是否能通过足迹检查，以 Nav2 路径服务的结果为准。
    """
    r,i=raw.info,inflated.info
    if (r.width,r.height,r.resolution,r.origin.position.x,r.origin.position.y)!=(i.width,i.height,i.resolution,i.origin.position.x,i.origin.position.y):
        return []
    a,b=grid_array(raw),grid_array(inflated)
    # 与终点入口共用同一阈值，避免终点放宽后中间观察点仍卡在旧值。
    max_cost=int(os.environ.get('FOREST_NAV_MAX_GOAL_COST','40'))
    rows,cols=np.where((a>=0)&(a<=max_cost)&(b>=0)&(b<99))
    xy=np.column_stack((r.origin.position.x+(cols+.5)*r.resolution,r.origin.position.y+(rows+.5)*r.resolution))
    if not len(xy):return []
    robot_dist=np.linalg.norm(xy-np.asarray(pose[:2]),axis=1)
    keep=(robot_dist>=.8)&(robot_dist<=4.)
    for old in excluded:keep &= np.linalg.norm(xy-np.asarray(old[:2]),axis=1)>.8
    xy,robot_dist=xy[keep],robot_dist[keep]
    if not len(xy):return []
    # 边界属于未知格，观察点退回已知区0.65~1.4米；身体安全余量由Nav2膨胀决定。
    distance=np.full(len(xy),np.inf);gain=np.zeros(len(xy))
    for frontier in frontiers:
        points=np.asarray(frontier['points'],dtype=float).reshape(-1,2)
        if not len(points):continue
        for batch in np.array_split(points,max(1,math.ceil(len(points)/128))):
            d=np.sqrt(((xy[:,None,:]-batch[None,:,:])**2).sum(axis=2)).min(axis=1)
            better=d<distance;distance[better]=d[better];gain[better]=min(frontier['size'],100)
    score=np.linalg.norm(xy-np.asarray(target[:2]),axis=1)+.3*robot_dist-.002*gain
    score[(distance<.65)|(distance>1.4)]=np.inf
    chosen=[]
    for index in np.argsort(score):
        if not np.isfinite(score[index]):break
        x,y=xy[index]
        if any(math.hypot(
                x-(c['point'][0] if with_metadata else c[0]),
                y-(c['point'][1] if with_metadata else c[1]))<.8
               for c in chosen):
            continue
        # 朝向未知边界的大致方向，便于到达后观测；360度LiDAR无需盲目原地旋转。
        yaw=math.atan2(target[1]-y,target[0]-x)
        candidate=(float(x),float(y),yaw)
        chosen.append(dict(
            point=list(candidate),
            score=float(score[index]),
            robot_distance_m=float(robot_dist[index]),
            frontier_distance_m=float(distance[index]),
            frontier_gain=float(gain[index]),
        ) if with_metadata else candidate)
        if len(chosen)>=limit:break
    return chosen
