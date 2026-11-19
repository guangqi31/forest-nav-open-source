"""从实际PhysX刚体反馈核对步态包络和非脚部接触，不参与策略或控制。

使用实际body pose与同一URDF碰撞体，计算当前姿态的保守水平包络。
每十步采样；不能声称覆盖采样间隙或所有未来步态。
"""
import json
from pathlib import Path
import numpy as np

def matrices(quaternions):
    x,y,z,w=np.asarray(quaternions).T
    return np.stack([1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),
        2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),
        2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)],axis=-1).reshape(-1,3,3)

class ForestSafetyAudit:
    def __init__(self,env,directory,ground_map):
        self.robot=env.unwrapped.scene['robot'];self.contacts=env.unwrapped.scene['contact_forces']
        self.directory=Path(directory);self.ground_map=ground_map
        profile=json.loads((self.directory/'preflight.json').read_text())
        self.shapes=profile['robot']['collision_shapes'];self.names=list(self.robot.body_names)
        missing={shape['link'] for shape in self.shapes}-set(self.names)
        if missing:raise RuntimeError(f'动态包络缺少PhysX刚体映射：{missing}')
        self.contact_indices=[i for i,name in enumerate(self.contacts.body_names) if 'ankle' not in name and 'foot' not in name]
        self.report=dict(samples=0,max_measured_radius_m=0.,max_nonfoot_contact_n=0.,min_root_height_m=100.,min_clearance_to_ground_m=100.,footprint_radius_m=.45,falls_observed=0)

    def tick(self,frame):
        if frame%10:return
        data=self.robot.data
        position=data.body_pos_w.torch[0].detach().cpu().numpy()
        rotations=matrices(data.body_quat_w.torch[0].detach().cpu().numpy())
        root=data.root_pos_w.torch[0].detach().cpu().numpy()
        maximum=0.
        for shape in self.shapes:
            index=self.names.index(shape['link']);matrix=rotations[index]
            center=position[index]+matrix@np.asarray(shape['center'])
            if shape['type']=='sphere':extent=np.repeat(shape['radius'],3)
            else:
                axis=matrix@np.asarray(shape['axis']);extent=np.abs(axis)*shape['length']/2+shape['radius']*np.sqrt(np.maximum(0,1-axis*axis))
            xy=np.maximum(np.abs(center[:2]-root[:2]-extent[:2]),np.abs(center[:2]-root[:2]+extent[:2]))
            maximum=max(maximum,float(np.linalg.norm(xy)))
        force=self.contacts.data.net_forces_w.torch[0].detach().cpu().numpy()
        nonfoot=float(np.linalg.norm(force[self.contact_indices],axis=1).max(initial=0.))
        self.report['samples']+=1
        self.report['max_measured_radius_m']=max(self.report['max_measured_radius_m'],maximum)
        self.report['max_nonfoot_contact_n']=max(self.report['max_nonfoot_contact_n'],nonfoot)
        self.report['min_root_height_m']=min(self.report['min_root_height_m'],float(root[2]))
        elevation,origin,resolution=self.ground_map
        col,row=np.floor((root[:2]-origin)/resolution).astype(int)
        if 0<=row<elevation.shape[0] and 0<=col<elevation.shape[1] and np.isfinite(elevation[row,col]):
            clearance=float(root[2]-elevation[row,col])
            self.report['min_clearance_to_ground_m']=min(self.report['min_clearance_to_ground_m'],clearance)
            if clearance<.45:self.report['falls_observed']+=1
        if frame%50==0:self.save()

    def save(self):
        self.report['nominal_footprint_contains_sampled_envelope']=self.report['max_measured_radius_m']<=self.report['footprint_radius_m']
        self.report['scope']='本次运动每0.2仿真秒采样的实际刚体包络，不是所有姿态证明'
        (self.directory/'safety_runtime_verification.json').write_text(json.dumps(self.report,ensure_ascii=False,indent=2)+'\n')
