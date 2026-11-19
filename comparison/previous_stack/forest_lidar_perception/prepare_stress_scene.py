from pathlib import Path
import sys
root=Path('/home/ubuntu/robot_projects/forest_nav')
sys.path.insert(0,str(root/'forest_terrain_3d'))
from build_scene import build
import geometry
import numpy as np
from geometry import SceneConfig
# 终点提供明确支撑平台；只修改真实几何，平台之间不铺导航走廊。
# 原复杂地形在平台外完整保留。出生区安全平台由现有生成器提供。
original_height=geometry.height
def support_height(x,y,cfg):
    z=original_height(x,y,cfg)
    distance=np.hypot(np.asarray(x)+27.,np.asarray(y))
    return z*geometry.smoothstep((distance-1.3)/.9)
geometry.height=support_height
original_scene=geometry.make_scene
def stress_scene(cfg):
    points,triangles,objects=original_scene(cfg)
    objects=[obj for obj in objects if np.hypot(obj['x']+27.,obj['y'])>3.]
    return points,triangles,objects
import build_scene
build_scene.make_scene=stress_scene
build(SceneConfig(preset='complex',size_m=64.,mesh_resolution_m=.125),root/'forest_terrain_3d/generated/stress')
sys.path.insert(0,str(root/'terrain_framework_baseline/adapters'))
from usd_to_pcl import export
export('stress')
