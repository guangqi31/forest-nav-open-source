"""验证LIO边界的时间、外参和评估方法；不复制上游滤波器实现。"""
import sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from forest_nav_control.lio_adapter import convert_cloud,LioAdapter

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'forest_lidar_perception'))
from lio_evaluation import evaluate


def test_cloud_layout_keeps_instantaneous_scan_and_real_ring():
    fields=[PointField(name=n,offset=o,datatype=d,count=1) for n,o,d in
        [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('ring',16,4),('time',20,7)]]
    msg=point_cloud2.create_cloud(Header(frame_id='lidar_link'),fields,[(1.,2.,3.,4.,31,0.),(2.,3.,4.,5.,0,0.)])
    out=convert_cloud(msg)
    points=point_cloud2.read_points(out)
    assert out.header.frame_id=='lidar_link' and out.point_step==28
    assert list(points['ring'])==[31,0] and np.all(points['timestamp']==0.)
    assert np.all(points['intensity']==[4.,5.])
    bad=point_cloud2.create_cloud(msg.header,fields,[(1.,2.,3.,4.,1,.02)])
    with pytest.raises(ValueError,match='瞬时扫描'):convert_cloud(bad)


def test_epoch_change_and_imu_gap_keep_adapter_running():
    adapter=SimpleNamespace(source=None,source_ready=False,last_status_wall=None)
    import json
    LioAdapter.on_source(adapter,SimpleNamespace(data=json.dumps(dict(session=None,clock_epoch=0,ready=False))))
    assert adapter.source is None
    def status(**extra):
        import json
        return SimpleNamespace(data=json.dumps(dict(session='a',clock_epoch=0,ready=True,**extra)))
    LioAdapter.on_source(adapter,status())
    assert adapter.source==('a',0)
    LioAdapter.on_source(adapter,status(imu_gaps=1))
    import json
    LioAdapter.on_source(adapter,SimpleNamespace(data=json.dumps(dict(session='a',clock_epoch=1,ready=True))))
    assert adapter.source==('a',1)


def test_evaluation_uses_fixed_first_pose_alignment_without_fitting_drift():
    # 真值LiDAR比IMU高0.65m；估计器局部坐标绕Z旋转90度并具有任意初始平移。
    truth=[];est=[]
    for i in range(50):
        truth.append([1000000000+i*200000000,i*.1,0.,1.65,0.,0.,0.,1.])
        est.append([truth[-1][0],10.,20.-i*.1,30.,0.,0.,-2**-.5,2**-.5])
    metrics,_=evaluate(est,truth)
    assert metrics['passed'] and metrics['position_rmse_m']<1e-10
    # 上游浮点秒截断损失1ns仍是同帧；1ms差异不能悄悄按最近邻配对。
    shifted=[list(row) for row in est]
    for row in shifted:row[0]-=1
    metrics,_=evaluate(shifted,truth)
    assert metrics['paired_poses']==50 and metrics['max_timestamp_numeric_skew_ns']==1
    for row in shifted:row[0]-=1000000
    assert evaluate(shifted,truth)[0]['paired_poses']==0
    est[-1][1]+=.9
    metrics,_=evaluate(est,truth)
    assert not metrics['passed'] and metrics['end_position_error_m']==pytest.approx(.9)
