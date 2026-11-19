"""新林地导航预检：资源一致性、URDF名义碰撞包络与传感器专用网格。

名义包络只解释当前圆形足迹的来源；不是任意关节姿态的动态扫掠包络。
碰撞几何由已验收的真实USD导出PLY合并，不能从代价图反推障碍。
"""
import ast,hashlib,json,math,re,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parent

def transform(origin):
    matrix=np.eye(4)
    if origin is not None:
        matrix[:3,:3]=Rotation.from_euler('xyz',[float(x) for x in origin.get('rpy','0 0 0').split()]).as_matrix()
        matrix[:3,3]=[float(x) for x in origin.get('xyz','0 0 0').split()]
    return matrix

def nominal_envelope():
    source=PROJECT/'InstinctLab/source/instinctlab/instinctlab/assets/unitree_g1.py'
    tree=ast.parse(source.read_text());angles=None
    # 从实际资产配置读取名义角，不复制一套容易过期的数值。
    for node in tree.body:
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Attribute) and ast.unparse(t)=='G1_29DOF_TORSOBASE_POPSICLE_CFG.init_state.joint_pos' for t in node.targets):angles=ast.literal_eval(node.value)
    assert angles is not None
    urdf=source.parent/'resources/unitree_g1/urdf/g1_29dof_torsobase_popsicle.urdf'
    robot=ET.parse(urdf).getroot();joints=robot.findall('joint');child_links={j.find('child').get('link') for j in joints}
    frames={link.get('name'):np.eye(4) for link in robot.findall('link') if link.get('name') not in child_links}
    remaining=list(joints)
    while remaining:
        count=len(remaining)
        for joint in remaining[:]:
            parent=joint.find('parent').get('link');child=joint.find('child').get('link')
            if parent not in frames:continue
            angle=next((v for pattern,v in angles.items() if re.fullmatch(pattern,joint.get('name'))),0.)
            matrix=transform(joint.find('origin'));motion=np.eye(4)
            if joint.get('type') in ('revolute','continuous'):
                axis=np.array([float(v) for v in joint.find('axis').get('xyz').split()]);motion[:3,:3]=Rotation.from_rotvec(axis/np.linalg.norm(axis)*angle).as_matrix()
            frames[child]=frames[parent]@matrix@motion;remaining.remove(joint)
        assert len(remaining)<count,'URDF关节树不连通'
    bounds=[];shapes=[]
    for link in robot.findall('link'):
        for collision in link.findall('collision'):
            local=transform(collision.find('origin'));matrix=frames[link.get('name')]@local;shape=collision.find('geometry')[0]
            description=dict(link=link.get('name'),type=shape.tag,center=local[:3,3].tolist(),axis=local[:3,2].tolist(),radius=float(shape.get('radius')))
            if shape.tag=='cylinder':description['length']=float(shape.get('length'))
            shapes.append(description)
            if shape.tag=='sphere':ext=np.repeat(float(shape.get('radius')),3)
            elif shape.tag=='cylinder':
                radius=float(shape.get('radius'));half=float(shape.get('length'))/2
                # 旋转后圆柱的精确轴对齐包围盒：轴向投影+横截面圆盘投影。
                axis=matrix[:3,2];ext=np.abs(axis)*half+radius*np.sqrt(np.maximum(0,1-axis*axis))
            else:raise ValueError('未支持的实际碰撞体 '+shape.tag)
            bounds.append((matrix[:3,3]-ext,matrix[:3,3]+ext))
    low=np.min([b[0] for b in bounds],axis=0);high=np.max([b[1] for b in bounds],axis=0)
    radius=max(np.linalg.norm([x,y]) for x in (low[0],high[0]) for y in (low[1],high[1]))
    assert radius+.08<=.45, f'0.45米足迹不足以覆盖名义包络与0.08米余量：{radius}'
    return dict(collision_shapes=shapes,urdf=str(urdf),urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest(),nominal_aabb_root_frame_m=[low.tolist(),high.tolist()],aabb_circumscribed_radius_m=float(radius),nominal_collision_height_m=float(high[2]-low[2]),nav2_radius_m=.45,remaining_nominal_margin_m=float(.45-radius),limitation='名义姿态碰撞包络，不保证步态摆臂/任意姿态全身包络')

def read_ply(path):
    with path.open() as f:
        vertices=faces=None
        while True:
            line=f.readline().strip()
            if line.startswith('element vertex '):vertices=int(line.split()[-1])
            if line.startswith('element face '):faces=int(line.split()[-1])
            if line=='end_header':break
            if not line:raise ValueError('PLY头无效')
        points=np.array([[float(v) for v in f.readline().split()] for _ in range(vertices)],dtype=np.float32)
        triangles=np.array([[int(v) for v in f.readline().split()] for _ in range(faces)],dtype=np.int32)
    assert np.all(triangles[:,0]==3)
    return points,triangles[:,1:]

def prepare(preset):
    folder=PROJECT/'terrain_framework_baseline/generated'/preset
    meta=json.loads((folder/'adapter_manifest.json').read_text())
    scene=PROJECT/'forest_terrain_3d/generated'/preset
    for name,digest in meta['source_hashes'].items():assert hashlib.sha256((scene/name).read_bytes()).hexdigest()==digest,'三维场景变更，必须重做转换'
    points=[];faces=[];offset=0
    for obj in meta['obstacles']:
        p,f=read_ply(folder/obj['file']);points.append(p);faces.append(f+offset);offset+=len(p)
    destination=ROOT/'results'/preset;destination.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(destination/'sensor_obstacles.npz',points=np.concatenate(points),triangles=np.concatenate(faces))
    envelope=nominal_envelope()
    report=dict(passed=True,preset=preset,robot=envelope,scene_id=meta['scene_id'],sensor_obstacle_primitives=len(points),sensor_canopies_included=False,ground_prim='/World/ground/terrain/Ground',map_world_translation_m=[0,0])
    (destination/'preflight.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');return report

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--preset',default='basic',choices=['basic','calibration','complex','stress']);a=p.parse_args()
    result=prepare(a.preset);result['robot']=dict(result['robot']);result['robot'].pop('collision_shapes',None)
    print(json.dumps(result,ensure_ascii=False,indent=2))
