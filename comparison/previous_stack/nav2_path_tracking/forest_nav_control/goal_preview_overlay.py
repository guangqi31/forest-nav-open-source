"""Isaac 候选目标显示层；不附加碰撞体，不修改机器人和导航数据。"""
import json
import math
import time
from .goal_preview import preview_file


class GoalPreviewOverlay:
    def __init__(self, ground_height):
        import omni.usd
        from pxr import Gf, Sdf, UsdGeom, UsdShade
        self.Gf = Gf
        self.ground_height = ground_height
        self.path = preview_file()
        self.next_read = 0.
        self.last_payload = None
        self.capture_due = None
        self.capture_done = False
        stage = omni.usd.get_context().get_stage()
        self.curve = UsdGeom.BasisCurves.Define(stage, '/World/ForestNavigation/CandidateGoal')
        self.curve.CreateTypeAttr('linear')
        self.curve.CreateWrapAttr('nonperiodic')
        self.curve.CreateWidthsAttr([.065])
        self.curve.SetWidthsInterpolation('constant')
        material = UsdShade.Material.Define(stage, '/World/ForestNavigation/CandidateMaterial')
        shader = UsdShade.Shader.Define(stage, '/World/ForestNavigation/CandidateMaterial/Shader')
        shader.CreateIdAttr('UsdPreviewSurface')
        self.color = shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f)
        self.emission = shader.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        UsdShade.MaterialBindingAPI.Apply(self.curve.GetPrim()).Bind(material)
        self.curve.MakeInvisible()

    def tick(self):
        """按墙钟 10 Hz 读最新预览；发送端失联后隐藏，避免遗留绿色结论。"""
        now = time.monotonic()
        if now < self.next_read:
            return
        self.next_read = now + .1
        # 按需截取一次实际视口，便于确认候选标记的颜色和落点，默认不截图。
        if self.capture_due is not None and now >= self.capture_due and not self.capture_done:
            import os
            capture = os.environ.get('FOREST_NAV_GOAL_CAPTURE')
            if capture:
                from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
                capture_viewport_to_file(get_active_viewport(), capture)
            self.capture_done = True
        try:
            payload = json.loads(self.path.read_text())
            if not 0 <= now - payload['sent_monotonic'] < 1.5 or not payload.get('visible'):
                self.curve.MakeInvisible()
                self.last_payload = None
                return
            key = tuple(payload.get(k) for k in ('x', 'y', 'z', 'yaw', 'status'))
            if key == self.last_payload:
                return
            x, y, z, yaw = (float(payload[k]) for k in ('x', 'y', 'z', 'yaw'))
            if not all(math.isfinite(v) for v in (x, y, z, yaw)):
                raise ValueError('候选位置非有限值')
            # 地形高度仅服务于贴地绘图，不用于停车准入或导航定位。
            def point(dx, dy, lift=0.):
                px, py = x + dx, y + dy
                try:
                    pz = self.ground_height(px, py)
                except ValueError:
                    pz = z + .12
                return self.Gf.Vec3f(px, py, pz + .08 + lift)
            # 0.45 米圆环沿用当前导航半径，只表达候选占地范围。
            ring = [point(.45*math.cos(a*math.tau/48), .45*math.sin(a*math.tau/48)) for a in range(49)]
            c, s = math.cos(yaw), math.sin(yaw)
            arrow = [point(0., 0.), point(.85*c, .85*s),
                     point(.60*c-.18*s, .60*s+.18*c), point(.85*c, .85*s),
                     point(.60*c+.18*s, .60*s-.18*c)]
            # 竖向标杆帮助从斜视视角定位，全部为无碰撞显示曲线。
            pole = [point(0., 0.), point(0., 0., .8)]
            color = {'allowed': (.05, 1., .2), 'blocked': (1., .05, .03),
                     'unknown': (1., .65, .02)}[payload['status']]
            self.color.Set(self.Gf.Vec3f(*color))
            self.emission.Set(self.Gf.Vec3f(*color))
            self.curve.GetPointsAttr().Set(ring + arrow + pole)
            self.curve.GetCurveVertexCountsAttr().Set([len(ring), len(arrow), len(pole)])
            self.curve.MakeVisible()
            self.last_payload = key
            # 显示回执只记录 USD 图元状态，不参与停车判断。
            self.path.with_suffix('.rendered.json').write_text(json.dumps(dict(
                world_pose=[x,y,z,yaw], status=payload['status'], visible=True,
                vertices=len(ring)+len(arrow)+len(pole), sent_monotonic=now)))
            if self.capture_due is None:
                self.capture_due = now + 1.
        except (OSError, ValueError, TypeError, KeyError):
            self.curve.MakeInvisible()
            self.last_payload = None
