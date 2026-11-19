"""核对终点准入的代价边界、地图坐标边界和地图更新语义。"""
import os
import unittest
from unittest.mock import patch
from nav_msgs.msg import OccupancyGrid
from forest_nav_control.goal_validation import GoalMapValidator,GoalRejected

class Node:
    def create_subscription(self,*args):return object()

class GoalValidationTests(unittest.TestCase):
    def setUp(self):
        self.env=patch.dict(os.environ,{'FOREST_TERRAIN_SCENE':'test-scene','FOREST_NAV_MAX_GOAL_COST':'25'})
        self.env.start();self.addCleanup(self.env.stop)
        self.validator=GoalMapValidator(Node())
        self.grid=OccupancyGrid();self.grid.header.frame_id='map'
        self.grid.info.width=4;self.grid.info.height=4;self.grid.info.resolution=.1
        self.grid.info.origin.position.x=-.2;self.grid.info.origin.position.y=-.2
        self.grid.info.origin.orientation.w=1.;self.grid.data=[0]*16
        self.validator.on_map(self.grid)

    def rejects(self,point,reason):
        with self.assertRaises(GoalRejected) as caught:self.validator.check(point)
        self.assertEqual(caught.exception.reason,reason)

    def test_cost_limit_inclusive_and_above(self):
        self.grid.data[0]=25;self.grid.data[1]=26
        self.assertEqual(self.validator.check([-.15,-.15])['cost'],25)
        self.rejects([-.05,-.15],'stopping_cost')

    def test_default_forty_matches_observation_candidates(self):
        from forest_nav_control.exploration_policy import observation_candidates
        from test_unknown_navigation import grid,frontiers
        with patch.dict(os.environ):
            os.environ.pop('FOREST_NAV_MAX_GOAL_COST',None)
            validator=GoalMapValidator(Node());validator.on_map(self.grid)
            self.grid.data[0]=40;self.grid.data[1]=41
            self.assertEqual(validator.max_cost,40)
            self.assertEqual(validator.check([-.15,-.15])['cost'],40)
            with self.assertRaises(GoalRejected) as error:validator.check([-.05,-.15])
            self.assertEqual(error.exception.reason,'stopping_cost')
            self.grid.data[0]=100
            with self.assertRaises(GoalRejected) as error:validator.check([-.15,-.15])
            self.assertEqual(error.exception.reason,'occupied')
            raw=grid();inflated=grid()
            raw.data=[40 if v>=0 else -1 for v in raw.data]
            self.assertTrue(observation_candidates(raw,inflated,frontiers(),(0,0),(3.5,0)))
            raw.data=[41 if v>=0 else -1 for v in raw.data]
            self.assertFalse(observation_candidates(raw,inflated,frontiers(),(0,0),(3.5,0)))

    def test_negative_origin_and_edges(self):
        self.assertEqual(self.validator.check([-.2,-.2])['row'],0)
        self.rejects([-.200001,0],'outside')
        self.rejects([.2,0],'outside')
        self.rejects([0,.2],'outside')

    def test_unknown_and_forbidden(self):
        self.grid.data[0]=-1;self.grid.data[1]=100
        self.rejects([-.15,-.15],'unknown');self.rejects([-.05,-.15],'occupied')

    def test_nonfinite(self):
        self.rejects([float('nan'),0],'nonfinite')
        self.rejects([0,float('inf')],'nonfinite')

    def test_new_map_invalidates_old_decision(self):
        self.validator.check([-.15,-.15])
        new=OccupancyGrid();new.header=self.grid.header;new.info=self.grid.info;new.data=[100]*16
        self.validator.on_map(new);self.rejects([-.15,-.15],'occupied')

    def test_rotated_and_missing_map_not_ready(self):
        self.grid.info.origin.orientation.z=.1;self.validator.on_map(self.grid)
        self.assertFalse(self.validator.ready);self.rejects([0,0],'map_not_ready')

    def test_exploration_outside_then_unknown_then_known_without_filling(self):
        # 远目标不对应现有单元。接纳只返回意图分类，原图尺寸和内容必须不变。
        target=[.35,0.]
        before=list(self.grid.data)
        result=self.validator.check(target,allow_unobserved=True)
        self.assertTrue(result['unknown']);self.assertTrue(result['outside'])
        self.assertIsNone(result['cost'])
        self.assertEqual(self.grid.info.width,4);self.assertEqual(list(self.grid.data),before)
        self.rejects(target,'outside')  # 严格/离线入口仍拒绝。
        # 用新观测发布的扩展图覆盖，目标先是未知，再变成可走，最后发现障碍。
        expanded=OccupancyGrid();expanded.header=self.grid.header
        expanded.info.resolution=.1;expanded.info.width=expanded.info.height=10
        expanded.info.origin.position.x=expanded.info.origin.position.y=-.5
        expanded.info.origin.orientation.w=1.;expanded.data=[-1]*100
        self.validator.on_map(expanded)
        result=self.validator.check(target,allow_unobserved=True)
        self.assertTrue(result['unknown']);self.assertFalse(result['outside'])
        index=5*10+8;expanded.data[index]=20
        result=self.validator.check(target,allow_unobserved=True)
        self.assertFalse(result['unknown']);self.assertEqual(result['cost'],20)
        for cost,reason in [(26,'stopping_cost'),(100,'occupied')]:
            expanded.data[index]=cost
            with self.assertRaises(GoalRejected) as caught:self.validator.check(target,allow_unobserved=True)
            self.assertEqual(caught.exception.reason,reason)
            self.assertEqual(caught.exception.cost,cost)
            self.assertEqual(caught.exception.max_stopping_cost,25)

    def test_exploration_still_requires_valid_map_and_finite_goal(self):
        with self.assertRaises(GoalRejected) as caught:self.validator.check([float('inf'),0],allow_unobserved=True)
        self.assertEqual(caught.exception.reason,'nonfinite')
        self.validator.grid=None
        with self.assertRaises(GoalRejected) as caught:self.validator.check([17,0],allow_unobserved=True)
        self.assertEqual(caught.exception.reason,'map_not_ready')

    def test_legacy_mode_keeps_existing_behavior(self):
        with patch.dict(os.environ,clear=True):
            validator=GoalMapValidator(Node())
            self.assertFalse(validator.enabled);self.assertTrue(validator.ready)
            self.assertIsNone(validator.check([17,0]))

if __name__=='__main__':unittest.main()
