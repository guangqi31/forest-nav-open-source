from types import SimpleNamespace as NS
from forest_nav_control.far_path import sample_route,route_length


def grid():
    return NS(header=NS(frame_id='map'),info=NS(width=100,height=100,resolution=.1,
        origin=NS(position=NS(x=-1.,y=-1.))),data=[0]*10000)


def test_corners_are_preserved_instead_of_cutting_through_obstacle():
    g=grid();p=sample_route([(0,0),(1,0),(1,1)])
    assert route_length(p)>1.99 and (1.,0.) in p
    assert all(y==0 or x==1 for x,y in p)


def test_map_does_not_truncate_far_route():
    for cost in (-1,100):
        g=grid();g.data[10*100+20]=cost
        p=sample_route([(0,0),(2,0)])
        assert p[-1]==(2.,0.)


def test_nonlethal_terrain_above_parking_threshold_can_be_traversed():
    g=grid();g.data=[60]*10000
    assert route_length(sample_route([(0,0),(2,0)]))>1.99


def test_route_geometry_is_preserved_for_native_controller():
    p=sample_route([(0,0),(2,0),(2,2)])
    assert p[0]==(0.,0.) and p[-1]==(2.,2.)


def test_robot_distance_does_not_reject_far_route():
    assert sample_route([(0,0),(2,0)])[-1]==(2.,0.)
