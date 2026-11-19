"""FAR折线加密；保留所有路段，不添加地图通行判断。"""
import math


def sample_route(points, spacing=.05):
    """输出完整折线，不读取地图或机器人到路线的距离。"""
    if len(points)<2:return []
    if not all(math.isfinite(v) for p in points for v in p[:2]):return []
    out=[tuple(points[0][:2])]
    for a,b in zip(points,points[1:]):
        n=max(1,math.ceil(math.dist(a[:2],b[:2])/spacing))
        out.extend((a[0]+(b[0]-a[0])*j/n,a[1]+(b[1]-a[1])*j/n) for j in range(1,n+1))
    return out


def route_length(points):
    return sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:]))
