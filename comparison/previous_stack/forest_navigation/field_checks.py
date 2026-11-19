"""林地实测的独立几何核对：使用转换地图和真实根节点轨迹。

这里不计算导航路线；路线由 Nav2 生成。检查整圆占地而非只检查中心点，
并把离散插值误差从净空中扣除。地图禁行格既包含障碍，也包含不宜行走的地形。
"""
import json
import math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree


def angle_error(actual, target):
    """跨越±π时仍取最短角度差。"""
    return abs(math.atan2(math.sin(actual-target), math.cos(actual-target)))


def dense_path(points, step=.02):
    points=np.asarray(points,dtype=float)
    if len(points)<2:return points.reshape(-1,2)
    return np.concatenate([np.linspace(a,b,max(2,math.ceil(np.linalg.norm(b-a)/step)+1))
                           for a,b in zip(points[:-1],points[1:])])


def point_segment_distance(point, a, b):
    ab=b-a
    t=np.clip(np.dot(point-a,ab)/max(np.dot(ab,ab),1e-12),0.,1.)
    return float(np.linalg.norm(point-(a+t*ab)))


def segment_distance(a,b,c,d):
    def cross(u,v):return float(u[0]*v[1]-u[1]*v[0])
    ab=b-a;cd=d-c;den=cross(ab,cd)
    if abs(den)>1e-10:
        t=cross(c-a,cd)/den;u=cross(c-a,ab)/den
        if 0<=t<=1 and 0<=u<=1:return 0.
    return min(point_segment_distance(a,c,d),point_segment_distance(b,c,d),
               point_segment_distance(c,a,b),point_segment_distance(d,a,b))


class FieldMap:
    def __init__(self, project, preset):
        project=Path(project)
        self.file=project/'terrain_framework_baseline/results'/preset/'terrain_costmap.npz'
        with np.load(self.file,allow_pickle=False) as data:
            self.cost=data['cost'];self.res=float(data['resolution']);self.origin=data['origin']
            self.slope=data['slope']
        self.manifest=json.loads((project/'forest_terrain_3d/generated'/preset/'scene_manifest.json').read_text())
        rows,cols=np.where((self.cost<0)|(self.cost>=100))
        self.blocked=np.column_stack((cols,rows))*self.res+self.origin+self.res/2
        self.tree=cKDTree(self.blocked)

    def goal_details(self,goal):
        col,row=np.floor((np.array(goal[:2])-self.origin)/self.res).astype(int)
        return dict(slope_deg=float(np.degrees(self.slope[row,col])),raw_cost=int(self.cost[row,col]),
                    scope='离线地图诊断；不代表当前策略可在任意该坡度上停稳')

    def values(self,points):
        cr=np.floor((np.asarray(points)-self.origin)/self.res).astype(int)
        if not np.all((cr[:,0]>=0)&(cr[:,1]>=0)&(cr[:,0]<self.cost.shape[1])&(cr[:,1]<self.cost.shape[0])):
            raise AssertionError('轨迹越出地图')
        return self.cost[cr[:,1],cr[:,0]]

    def check_path(self,points,start,goal,radius=.45):
        samples=dense_path(points)
        assert len(samples)>1,'没有足够轨迹点'
        costs=self.values(samples)
        # 最靠近格心的禁行格不一定最靠近格子边缘。先找到最近格心距离d，
        # 再搜索d+半格对角线内所有格子，精确计算到每个方格AABB的距离。
        # 更远的格子按三角不等式不可能改进这个最小距离。
        dist,_=self.tree.query(samples)
        near=self.tree.query_ball_point(samples,dist+self.res*math.sqrt(2)/2+1e-9)
        clearance=float('inf')
        for point,indices in zip(samples,near):
            delta=np.maximum(np.abs(self.blocked[indices]-point)-self.res/2,0.)
            clearance=min(clearance,float(np.linalg.norm(delta,axis=1).min())-radius)
        # 线段按不超过0.02m插值；距离函数是1-Lipschitz，再扣0.01m覆盖间隙。
        clearance-=.01
        a=np.asarray(start[:2]);b=np.asarray(goal[:2]);axis=b-a
        lateral=float(np.max(np.abs((samples[:,0]-a[0])*axis[1]-(samples[:,1]-a[1])*axis[0]))/max(np.linalg.norm(axis),1e-9))
        return dict(sample_count=len(samples),raw_max_cost=int(costs.max()),
                    blocked_or_unknown_samples=int(np.sum((costs<0)|(costs>=100))),
                    footprint_radius_m=radius,swept_clearance_lower_bound_m=clearance,
                    max_lateral_deviation_m=lateral,
                    length_m=float(np.linalg.norm(np.diff(samples,axis=0),axis=1).sum()))

    def direct_obstruction(self,start,goal,object_id):
        line=dense_path([start[:2],goal[:2]])
        costs=self.values(line)
        obj=next(o for o in self.manifest['objects'] if o['id']==object_id)
        a=np.array(start[:2]);b=np.array(goal[:2]);center=np.array([obj['x'],obj['y']])
        if obj['kind']=='tree':distance=point_segment_distance(center,a,b)
        elif obj['kind']=='log':
            axis=np.array(obj['direction'][:2]);axis=axis/np.linalg.norm(axis)*obj['length']/2
            distance=segment_distance(a,b,center-axis,center+axis)
        else:raise ValueError('当前强制绕障用例只使用真实树干和倒木')
        return dict(object_id=object_id,kind=obj['kind'],axis_distance_m=distance,
                    object_radius_m=obj['radius'],straight_intersects_object=distance<obj['radius'],
                    straight_blocked_samples=int(np.sum(costs>=100)))
