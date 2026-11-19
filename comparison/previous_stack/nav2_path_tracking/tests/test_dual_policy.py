"""双策略过渡与停车判定检查；合成推理不代替Isaac站立/切换验收。"""
import importlib.util
import math
from pathlib import Path
import unittest
from forest_nav_control.dual_policy import StopTransition, parking_result, check_contract


class TransitionTests(unittest.TestCase):
    def test_high_speed_zero_command_does_not_switch_to_stand(self):
        mode=StopTransition()
        for _ in range(100):status=mode.update((0.,0.,0.),.4,0.,0.,.02)
        self.assertEqual(status['mode'],'braking')
        self.assertEqual(status['stand_weight'],0.)
        self.assertFalse(status['settled'])

    def test_stand_blend_settle_and_resume(self):
        mode=StopTransition();weights=[]
        for _ in range(110):
            status=mode.update((0.,0.,0.),0.,0.,0.,.02);weights.append(status['stand_weight'])
        self.assertTrue(status['settled']);self.assertEqual(status['mode'],'standing')
        self.assertLessEqual(max(b-a for a,b in zip(weights,weights[1:])),.051)
        first=mode.update((.2,0.,0.),0.,0.,0.,.02)
        self.assertEqual(first['mode'],'starting');self.assertFalse(first['settled'])
        for _ in range(15):status=mode.update((.2,0.,0.),0.,0.,0.,.02)
        self.assertEqual(status['mode'],'walking');self.assertEqual(status['stand_weight'],0.)

    def test_pose_and_speed_independent(self):
        self.assertEqual(parking_result((1,0,0),(1,0,0),{'settled':False})[0],'settling')
        self.assertEqual(parking_result((1,0,0),(1.4,0,0),{'settled':True})[0],'failed')
        self.assertEqual(parking_result((1,0,0),(1.1,0,0),{'settled':True})[0],'verified')
        self.assertEqual(parking_result((1,0,math.pi),(1,0,-math.pi),{'settled':True})[0],'verified')

    def test_bad_state_is_explicit_error(self):
        with self.assertRaises(ValueError):StopTransition().update((0,0,0),float('nan'),0,0,.02)


@unittest.skipUnless(importlib.util.find_spec('torch') and importlib.util.find_spec('onnxruntime'),
                     '真实ONNX接口检查在Isaac Python中单独运行')
class ModelTests(unittest.TestCase):
    def test_actual_stand_models_and_history(self):
        import torch
        import numpy as np
        from forest_nav_control.dual_policy import DualPolicy
        root=Path('/home/ubuntu/instintlab-gpu/work/instinctlab_models/data&model/checkpoints')
        segments={name:(24 if i<3 else 232,) for i,name in enumerate(
            ('base_ang_vel','projected_gravity','velocity_commands','joint_pos','joint_vel','actions'))}
        policy=DualPolicy(lambda obs:torch.zeros(1,29),root/'parkour_onboard_preview_stair',root/'stand_onboard',segments)
        obs=torch.zeros(1,768);obs[:,48+2:72:3]=-1.
        for _ in range(40):actions=policy(obs,(0,0,0),0,0,0,.02)
        self.assertEqual(tuple(actions.shape),(1,29));self.assertTrue(torch.isfinite(actions).all())
        self.assertGreater(np.linalg.norm(policy.latent),0.)
        self.assertTrue(np.all(policy.history['velocity_commands']==0))
        policy(obs,(.2,0,0),0,0,0,.02)
        self.assertEqual(policy.status['mode'],'walking' if policy.transition.direct_switch else 'starting')


if __name__=='__main__':unittest.main()
