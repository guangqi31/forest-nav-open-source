import numpy as np
from types import SimpleNamespace
from forest_nav_control.terrain_rebuild import TerrainEvidence
from forest_nav_control.lio_costmap import LioCostmap
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

def value(grid,x,y):
    col,row=np.floor((np.array([x,y])-grid.origin)/grid.resolution).astype(int)
    return int(grid.cost[row,col])

def test_classes_preserved_and_reprojected():
    e=TerrainEvidence(classified=True)
    e.observe([[1.1,.1,0,100],[2.1,.1,0,0]],1.)
    assert value(e.global_grid,1.1,.1)==100
    assert value(e.global_grid,2.1,.1)==0
    assert value(e.global_grid,3.1,.1)==-1
    e.correct([1.,0.,0.],1,1.)
    assert value(e.global_grid,2.1,.1)==100
    assert value(e.global_grid,3.1,.1)==0

def test_far_topics_override_height_and_obstacle_wins_same_cell():
    calls=[]
    node=SimpleNamespace(classified_clouds={},observe_points=lambda p,s:calls.append((p,s)))
    fields=[PointField(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z','intensity'))]
    h=Header();h.frame_id='lio_odom';h.stamp.sec=1
    # A tall free point and a low obstacle keep FAR's classification, irrespective of height/intensity.
    free=point_cloud2.create_cloud(h,fields,[(1.1,.1,5.,9.),(2.1,.1,5.,9.)])
    obs=point_cloud2.create_cloud(h,fields,[(1.1,.1,.01,0.)])
    LioCostmap.on_classified(node,free,0)
    assert not calls
    LioCostmap.on_classified(node,obs,100)
    points,stamp=calls[0]
    e=TerrainEvidence(classified=True);e.observe(points,stamp)
    assert value(e.global_grid,1.1,.1)==100
    assert value(e.global_grid,2.1,.1)==0
    # Subsequent explicit free classification clears an observed obstacle.
    e.observe([[1.1,.1,0,0]],2.)
    assert value(e.global_grid,1.1,.1)==0
