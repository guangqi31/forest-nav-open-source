"""FAR客户端不能等待已停用的Smac或BT服务。"""
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock
import time
from forest_nav_control.navigation_client import NavigationClient
from forest_nav_control.task_client import TaskClient


def done(value):
    f=Future();f.set_result(value);return f


def test_far_wait_ready_uses_only_controller_lifecycle_and_follow_path(monkeypatch):
    import forest_nav_control.navigation_client as module
    monkeypatch.setenv('FOREST_NAV_PLANNER','far')
    monkeypatch.setattr(module.rclpy,'ok',lambda:True)
    monkeypatch.setattr(module.rclpy,'spin_once',lambda *a,**k:None)
    services=[]
    def create(kind,name):
        services.append(name)
        return NS(wait_for_service=lambda **k:True,call_async=lambda req:done(NS(current_state=NS(id=3))))
    fake=NS(odom=object(),status={'ready':True},status_received=time.monotonic(),
        tf=NS(can_transform=lambda *a:True),create_client=create,destroy_client=lambda c:None,
        future=lambda f,**k:f.result(),planner=NS(wait_for_server=Mock(side_effect=AssertionError('Smac must be off'))),
        follower=NS(wait_for_server=lambda **k:True))
    NavigationClient.wait_ready(fake,timeout=1.)
    assert services==['/controller_server/get_state']


def test_far_submit_does_not_wait_for_bt_navigator(monkeypatch):
    monkeypatch.setenv('FOREST_NAV_PLANNER','far');monkeypatch.setenv('FOREST_NAV_MAP_MODE','lidar')
    import rclpy
    monkeypatch.setattr(rclpy,'ok',lambda:True)
    from rclpy.time import Time
    services=[];handle=NS(accepted=True,goal_id=NS(uuid=bytes(16)),get_result_async=lambda:Future())
    def create(kind,name):
        services.append(name)
        return NS(wait_for_service=lambda **k:True,call_async=lambda req:done(NS(current_state=NS(id=3))))
    fake=NS(wait_ready=lambda:None,create_client=create,destroy_client=lambda c:None,
        future=lambda f,**k:f.result(),spin_for=lambda t:None,goal_validator=NS(check=lambda *a,**k:None),
        navigator=NS(wait_for_server=lambda **k:True,send_goal_async=lambda *a,**k:done(handle)),
        records=NS(accepted=Mock()),get_clock=lambda:NS(now=lambda:Time(seconds=1.)),
        on_feedback=Mock(),on_task_result=Mock())
    assert TaskClient._submit(fake,(1.,0.,0.),'task') is handle
    assert services==['/controller_server/get_state']


def test_coordinate_panel_uses_controller_lifecycle_without_opening_window(monkeypatch):
    import rclpy
    from forest_nav_control.coordinate_controller import CoordinateController
    monkeypatch.setenv('FOREST_NAV_PLANNER','far');monkeypatch.setenv('FOREST_NAV_MAP_MODE','lidar')
    rclpy.init();node=CoordinateController()
    try:assert node.lifecycle.srv_name=='/controller_server/get_state'
    finally:node.destroy_node();rclpy.shutdown()
