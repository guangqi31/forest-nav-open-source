"""第3步：跨越旧边界、保留世界坐标、修正事务和原始测量重投影。"""
import json
import math
from types import SimpleNamespace
import numpy as np
import pytest
from forest_nav_control.terrain_costmap import TerrainGrid
from forest_nav_control.terrain_rebuild import TerrainEvidence
from forest_nav_control.lio_costmap import LioCostmap


def value(grid,x,y):
    col,row=np.floor((np.array([x,y])-grid.origin)/grid.resolution).astype(int)
    return int(grid.cost[row,col])


def test_expand_preserves_world_cells_unknown_timestamps_and_last_good_on_failure():
    grid=TerrainGrid(ttl=None,expand=True)
    grid.update([[.1,.1,0,.3]],1.)
    grid.update([[20.1,-17.1,0,0]],2.)
    assert grid.size>32 and value(grid,.1,.1)==30 and value(grid,20.1,-17.1)==0
    assert value(grid,-20,20)==-1
    grid.update([[130.1,0.,0.,0.]],3.)
    assert grid.size>256 and value(grid,130.1,0.)==0
    assert value(grid,.1,.1)==30


def test_rebuild_translation_rotation_no_ghost_and_absolute_not_incremental():
    evidence=TerrainEvidence()
    evidence.observe([[1.1,.1,0,.3],[2.1,.1,0,0],[20.1,.1,0,.3]],1.)
    evidence.correct([2.,3.,math.pi/2],1,1.)
    assert value(evidence.global_grid,1.9,4.1)==30
    assert value(evidence.global_grid,1.9,5.1)==0
    assert value(evidence.global_grid,1.1,.1)==-1
    assert value(evidence.global_grid,1.9,23.1)==30
    evidence.correct([0,0,0],2,1.)
    assert value(evidence.global_grid,1.1,.1)==30
    assert value(evidence.global_grid,1.9,4.1)==-1
    assert evidence.generation==2


def test_corrected_future_observation_clears_obstacle_without_reintroducing_old_position():
    evidence=TerrainEvidence()
    evidence.observe([[1.1,.1,0,.4]],1.)
    evidence.correct([4,0,0],1,1.)
    evidence.observe([[1.1,.1,0,0]],2.)
    assert value(evidence.global_grid,5.1,.1)==0
    assert value(evidence.global_grid,1.1,.1)==-1
    assert not evidence.observe([[1.1,.1,0,.4]],1.)
    assert value(evidence.global_grid,5.1,.1)==0


def test_bad_revision_or_oversized_correction_is_atomic():
    evidence=TerrainEvidence();evidence.observe([[1.1,.1,0,.4]],1.)
    evidence.correct([1,0,0],2,1.)
    old=evidence.global_grid.cost.copy()
    for transform,revision in [([2,0,0],1),([2,0,float('nan')],3),([0,0,0],True)]:
        with pytest.raises(ValueError):evidence.correct(transform,revision,1.)
        assert np.array_equal(evidence.global_grid.cost,old)
        assert evidence.revision==2 and evidence.generation==1


def test_local_expiry_and_global_history_survive_rebuild():
    evidence=TerrainEvidence();evidence.observe([[.1,.1,0,.4]],1.)
    evidence.observe([[2.1,.1,0,0]],5.)
    evidence.correct([1,0,0],1,5.)
    assert value(evidence.global_grid,1.1,.1)==40
    assert value(evidence.local_grid,1.1,.1)==40
    assert value(evidence.local_grid,3.1,.1)==0


def test_observation_cache_does_not_reject_after_old_limit():
    evidence=TerrainEvidence()
    evidence.observe([[.1,.1,0,0]],1.)
    assert evidence.observe([[1.1,.1,0,0]],2.)
    assert len(evidence.groups)==2 and evidence.last_stamp==2.


def test_correction_requires_current_session_and_expected_frames():
    evidence=TerrainEvidence();evidence.observe([[.1,.1,0,.4]],1.)
    obj=SimpleNamespace(source=('current',0),evidence=evidence,last_correction=None)
    data=dict(schema_version=1,session='old',clock_epoch=0,revision=1,
        frame_id='lio_map',child_frame_id='lio_odom',map_to_odom=[1,0,0])
    LioCostmap.on_correction(obj,SimpleNamespace(data=json.dumps(data)))
    assert not obj.last_correction['accepted'] and evidence.revision==0
    data.update(session='current',frame_id='map')
    LioCostmap.on_correction(obj,SimpleNamespace(data=json.dumps(data)))
    assert not obj.last_correction['accepted'] and evidence.revision==0
