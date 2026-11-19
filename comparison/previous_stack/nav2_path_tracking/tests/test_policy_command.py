"""检查策略真正读取的命令观察及环境命令缓冲同时更新，不只停留在 ROS 缓存。"""
import unittest
import math
from types import SimpleNamespace
import torch
from forest_nav_control.isaac_runtime import IsaacNavigation


class PolicyCommandTests(unittest.TestCase):
    def test_measured_ground_height_uses_height_scanners_only(self):
        runtime=IsaacNavigation.__new__(IsaacNavigation)
        def scanner(values):
            return SimpleNamespace(data=SimpleNamespace(
                ray_hits_w=SimpleNamespace(torch=torch.tensor([values],dtype=torch.float32))))
        runtime.left_height_scanner=scanner([[0.,0.,.1],[0.,0.,.2]])
        runtime.right_height_scanner=scanner([[0.,0.,.15],[float('inf'),0.,0.]])
        self.assertAlmostEqual(runtime.measured_ground_height(),.15,places=5)
        runtime.left_height_scanner=scanner([[float('inf'),0.,0.]])
        runtime.right_height_scanner=scanner([[float('inf'),0.,0.]])
        self.assertIsNone(runtime.measured_ground_height())

    def test_external_command_reaches_manager_and_actor_observation(self):
        runtime=IsaacNavigation.__new__(IsaacNavigation)
        runtime.endpoint=SimpleNamespace(read_command=lambda:(.2,0.,.3),
                                         read_path=lambda:((),-1))
        runtime.path_overlay=None
        runtime.command_term=SimpleNamespace(vel_command_b=torch.zeros(1,3))
        obs=torch.full((1,30),-1.)
        runtime.before_step(obs,(slice(3,27),(24,)))
        torch.testing.assert_close(runtime.command_term.vel_command_b,torch.tensor([[.2,0.,.3]]))
        torch.testing.assert_close(obs[:,3:27],torch.tensor([[.2,0.,.3]]).repeat(1,8))
        self.assertTrue(torch.all(obs[:,:3]==-1.))
        self.assertTrue(torch.all(obs[:,27:]==-1.))

    def test_xyzw_pose_and_horizontal_velocity(self):
        # 必须覆盖真实后端的xyzw顺序，不能只检查接收端的命令缓存。
        for yaw, translation in [(0., (-8., -2.)), (math.pi/2, (-8., -2.)), (0., (0., 0.)), (math.pi/2, (0., 0.))]:
            captured = []
            runtime = IsaacNavigation.__new__(IsaacNavigation)
            runtime.frames = 0
            runtime.map_translation = translation
            runtime.ground_map = None
            runtime.env = SimpleNamespace(unwrapped=SimpleNamespace(step_dt=.02))
            runtime.endpoint = SimpleNamespace(publish_state=lambda *a, **k: captured.append((a, k)))
            def array(value):
                return SimpleNamespace(torch=torch.tensor([value], dtype=torch.float32))
            runtime.robot = SimpleNamespace(data=SimpleNamespace(
                root_pos_w=array([0., 0., .82]),
                root_quat_w=array([0., 0., math.sin(yaw / 2), math.cos(yaw / 2)]),
                root_lin_vel_w=array([math.cos(yaw)*.2, math.sin(yaw)*.2, 0.]),
                root_ang_vel_w=array([0., 0., .1])))
            ground_scan=SimpleNamespace(data=SimpleNamespace(
                ray_hits_w=SimpleNamespace(torch=torch.tensor([[[0.,0.,0.]]]))))
            runtime.left_height_scanner=ground_scan
            runtime.right_height_scanner=ground_scan
            runtime.publish_state()
            args, kwargs = captured[-1]
            self.assertEqual(tuple(args[1][:2]), translation)
            self.assertAlmostEqual(args[1][2], yaw, places=5)
            self.assertAlmostEqual(args[2][0], .2, places=5)
            self.assertAlmostEqual(args[2][1], 0., places=5)
            self.assertEqual(kwargs['fault'], '')


if __name__=='__main__':
    unittest.main()
