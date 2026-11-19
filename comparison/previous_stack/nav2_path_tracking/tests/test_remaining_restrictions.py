from pathlib import Path
from types import SimpleNamespace as NS
import importlib.util,json,random,time
import numpy as np
import rclpy
from forest_nav_control.lio_adapter import LioAdapter
from forest_nav_control.lio_costmap import LioCostmap
from forest_nav_control.observation_health import ProgressClock
from forest_nav_control.cloud_wire import encode_chunks,CloudAssembler
from forest_nav_control.terrain_rebuild import TerrainEvidence
from forest_nav_control.coordinate_controller import CoordinateController


def test_paused_simulation_does_not_reject_progress_file(tmp_path):
    p=tmp_path/'clock.json';p.write_text(json.dumps(dict(stamp=1.,sent_monotonic=1.,session='test')))
    clock=ProgressClock(p)
    assert clock.poll(1.) and clock.poll(100.)
    p.write_text(json.dumps(dict(stamp=.5,sent_monotonic=101.,session='test')))
    assert clock.poll(101.) and clock.epoch==1


def test_large_delayed_cloud_roundtrip_in_shuffled_datagrams():
    points=np.arange(90000,dtype=np.float32).reshape(-1,3)
    chunks=encode_chunks(points,1.);assert len(chunks)>1 and max(map(len,chunks))<=60000
    random.Random(1).shuffle(chunks);assembler=CloudAssembler();complete=[]
    for chunk in chunks:
        result=assembler.feed(chunk)
        if result is not None:complete.append(result)
    assert len(complete)==1 and complete[0][0]==1.
    np.testing.assert_array_equal(complete[0][1],points)


def test_nested_lio_launch_has_no_cascade_exit_handler(monkeypatch,tmp_path):
    from launch.actions import RegisterEventHandler
    root=Path(__file__).resolve().parents[2]
    monkeypatch.setenv('FOREST_LIO_OUTPUT',str(tmp_path))
    spec=importlib.util.spec_from_file_location('nested_lio',root/'forest_lidar_perception/lio.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    actions=module.generate_launch_description().entities
    assert len(actions)==5 and not any(isinstance(a,RegisterEventHandler) for a in actions)


def test_gap_and_source_change_reach_costmap_without_process_failure(monkeypatch,tmp_path):
    monkeypatch.setenv('FOREST_LIO_OUTPUT',str(tmp_path));rclpy.init()
    adapter=LioAdapter();mapper=LioCostmap()
    try:
        mapper.evidence.observe([[1.,1.,0.,0.]],1.)
        for epoch in (0,1):
            adapter.on_source(NS(data=json.dumps(dict(session='a',clock_epoch=epoch,ready=False,imu_gaps=2))))
            mapper.on_lio(NS(data=json.dumps(dict(source=list(adapter.source),ready=False))))
        assert adapter.source==('a',1) and mapper.source==('a',1)
        assert mapper.evidence.observe([[2.,2.,0.,0.]],.5)
        assert mapper.evidence.last_stamp==.5
    finally:
        mapper.destroy_node();adapter.destroy_node();rclpy.shutdown()


def test_local_map_is_persistent_until_explicit_reset():
    e=TerrainEvidence();e.observe([[.1,.1,0.,0.]],1.)
    assert not e.local_grid.expire(100.)
    assert (e.local_grid.cost>=0).sum()==1


def test_panel_submission_not_gated_by_old_heartbeat():
    fake=NS(goal_validator=NS(ready=True),pose=[0.,0.,0.],navigator=NS(server_is_ready=lambda:True),
        far_received=0.,far_status=dict(ready=False),bridge_received=0.,bridge=dict(ready=False),
        lifecycle_active=False,lifecycle_checked=0.)
    assert CoordinateController.ready.fget(fake)


def test_long_preview_json_transport_has_no_route_length_rejection():
    root=Path(__file__).resolve().parents[2]
    spec=importlib.util.spec_from_file_location('preview_wire',root/'forest_route_preview/wire.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    payload=dict(seq=1,state='success',points=[[i*.1,i*.2] for i in range(10000)])
    chunks=module.encode_packets(payload);assert len(chunks)>1 and max(map(len,chunks))<65507
    decoder=module.Assembler();out=None
    for chunk in reversed(chunks):
        value=decoder.feed(chunk)
        if value is not None:out=value
    assert out==payload


def test_relative_height_has_no_hard_018m_lethal_cutoff():
    from forest_nav_control.terrain_costmap import TerrainGrid
    g=TerrainGrid(ttl=None)
    g.update([[.1,.1,0.,.17],[.5,.1,0.,.18],[.9,.1,0.,.4]],1.)
    costs=sorted(int(v) for v in g.cost[g.cost>=0])
    assert costs==[17,18,40] and 100 not in costs


