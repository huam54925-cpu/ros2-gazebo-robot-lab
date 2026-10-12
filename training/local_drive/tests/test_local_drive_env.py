import math
import unittest
from dataclasses import replace
import numpy as np
from training.envs.contracts import Frame, InfrastructureError
from training.envs.local_drive_env import LocalDriveEnv

class Backend:
    def __init__(self):self.seeds=[];self.fail=False;self.closed=0
    def reset(self,*,seed,scenario):
        self.seeds.append(seed)
        self.frame=Frame(len(self.seeds),0,900.,dict(lidar=900.,odom=900.,truth=900.),
            dict(lidar=dict(ranges=[2.]*360,range_min=.08,range_max=10.),odom_velocity=[0.,0.],goal_relative=[.15,0.,1.]),
            dict(truth_dist=3.,truth_pose=[0.,0.,0.],roll=0.,pitch=0.),(0.,0.,math.pi/2))
        return self.frame
    def advance(self,v,w,dt):
        if self.fail:raise InfrastructureError('original sample failure')
        self.frame=replace(self.frame,action_id=self.frame.action_id+1,sim_time=self.frame.sim_time+.2,odom_pose=(0.,.06,math.pi/2))
        return self.frame
    def stop(self):pass
    def close(self):
        self.closed+=1
        if self.fail:raise RuntimeError('stop failed')
class EnvTests(unittest.TestCase):
    def test_history_is_current_and_previous_body_frame(self):
        e=LocalDriveEnv(Backend());e.reset(seed=8)
        obs,_,_,truncated,_=e.step(np.array([.25,-.5],dtype=np.float32))
        np.testing.assert_allclose(obs[-5:],[.25,-.5,.5,0,0],atol=1e-6)
        self.assertFalse(truncated)
        obs,_=e.reset();np.testing.assert_array_equal(obs[-20:],0)
    def test_rng_none_continues(self):
        b=Backend();e=LocalDriveEnv(b)
        e.reset(seed=23);a=b.seeds[-1];e.reset();c=b.seeds[-1]
        e.reset(seed=23);self.assertEqual(b.seeds[-1],a)
        e.reset();self.assertEqual(b.seeds[-1],c);self.assertNotEqual(a,c)
    def test_missing_truth_fails_and_cleans_up(self):
        b=Backend();e=LocalDriveEnv(b);e.reset(seed=0)
        b.frame.evaluation.pop('truth_dist')
        with self.assertRaisesRegex(InfrastructureError,'truth'):e.step(np.zeros(2))
        self.assertGreater(b.closed,0)
    def test_cleanup_does_not_mask_original(self):
        b=Backend();e=LocalDriveEnv(b);e.reset(seed=0);b.fail=True
        with self.assertRaisesRegex(InfrastructureError,'original sample') as caught:e.step(np.zeros(2))
        self.assertTrue(any('stop failed' in x for x in caught.exception.__notes__))
    def test_relative_time_limit(self):
        b=Backend();e=LocalDriveEnv(b);e.reset(seed=0)
        b.frame=replace(b.frame,sim_time=1019.8)
        _,_,_,truncated,_=e.step(np.zeros(2));self.assertTrue(truncated)
    def test_arrival_reward_is_terminal_once(self):
        b=Backend();e=LocalDriveEnv(b);e.reset(seed=0)
        b.frame=replace(b.frame,evaluation={**b.frame.evaluation,'truth_dist':.3})
        _,r,terminated,_,_=e.step(np.zeros(2));self.assertTrue(terminated);self.assertGreater(r,10)
        with self.assertRaises(RuntimeError):e.step(np.zeros(2))
