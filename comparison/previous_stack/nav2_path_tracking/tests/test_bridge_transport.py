"""速度与显示传输隔离，以及独立定位新鲜度回归。"""
import json
import time
from types import SimpleNamespace as NS
import pytest
from forest_nav_control.bridge_node import SimBridge
from forest_nav_control.sim_endpoint import MAX_DATAGRAM_BYTES


def bridge_fixture():
    now=time.monotonic();sent=[];statuses=[]
    class Socket:
        fail_path=False
        def recvfrom(self,*args):raise BlockingIOError
        def sendto(self,data,destination):
            packet=json.loads(data)
            if self.fail_path and packet.get('kind')=='path':raise OSError('display send failed')
            sent.append((data,packet))
    obj=NS(socket=Socket(),use_lio=True,state_received=now,command_received=now,
        state=dict(ready=True,session='hardware',pose=[999.,888.,0.],sim_time=10.),
        command=(.2,0.,0.),localization=dict(ready=True,pose=[1.,2.,0.],stamp=10.),
        localization_received=now,sequence=0,destination=('127.0.0.1',0),
        plan_dirty=True,plan_points=[[i*.123456789,i*.987654321] for i in range(4096)],
        plan_revision=2,plan_last_sent=0.,status_pub=NS(publish=lambda msg:statuses.append(json.loads(msg.data))))
    return obj,sent,statuses


def test_route_is_separate_and_bounded_with_speed_sent_first():
    obj,sent,statuses=bridge_fixture();SimBridge._tick(obj)
    assert sent[0][1]['command']==[.2,0.,0.] and 'path' not in sent[0][1]
    assert sent[1][1]['kind']=='path' and len(sent[1][0])<=MAX_DATAGRAM_BYTES
    assert sent[1][1]['path'][0]==obj.plan_points[0]
    assert sent[1][1]['path'][-1]==obj.plan_points[-1]
    assert statuses[-1]['localization_ready']


def test_display_send_failure_does_not_lose_speed_or_kill_bridge():
    obj,sent,statuses=bridge_fixture();obj.socket.fail_path=True
    SimBridge._tick(obj)
    assert sent[0][1]['command']==[.2,0.,0.]
    assert statuses[-1]['path_send_error']=='display send failed' and obj.plan_dirty


@pytest.mark.parametrize('changes', [dict(ready=False),dict(stamp=9.),dict(stamp=None)])
def test_invalid_lio_measurement_does_not_override_speed_despite_fresh_hardware(changes):
    obj,sent,statuses=bridge_fixture();obj.localization.update(changes)
    SimBridge._tick(obj)
    assert sent[0][1]['command']==[.2,0.,0.]
    assert statuses[-1]['hardware_ready'] and not statuses[-1]['localization_ready']


def test_stopped_localization_status_does_not_override_speed_even_if_cached_ready_is_true():
    obj,sent,statuses=bridge_fixture();obj.localization_received-=2.
    SimBridge._tick(obj)
    assert sent[0][1]['command']==[.2,0.,0.]
    assert not statuses[-1]['localization_ready']
