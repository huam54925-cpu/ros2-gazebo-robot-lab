import unittest
from dataclasses import replace
from training.envs.contracts import Frame, InfrastructureError, decode_action, encode_lidar, validate_transition

class ContractTests(unittest.TestCase):
    def setUp(self):
        self.a=Frame(1,0,0.,{}, {}, {})
        self.b=Frame(1,1,.2,dict(lidar=.2,odom=.2,truth=.2),
                     dict(lidar=[],odom_velocity=[],goal_relative=[],history=[]),dict(truth_pose=[0,0,0]))
    def test_valid(self): validate_transition(self.a,self.b)
    def test_stale(self):
        with self.assertRaises(InfrastructureError): validate_transition(self.a,replace(self.b,sensor_times=dict(lidar=.1,odom=.2,truth=.2)))
    def test_episode(self):
        with self.assertRaises(InfrastructureError): validate_transition(self.a,replace(self.b,episode_id=2))
    def test_action_ack(self):
        with self.assertRaises(InfrastructureError): validate_transition(self.a,replace(self.b,action_id=0))
    def test_truth_leak(self):
        with self.assertRaises(InfrastructureError): validate_transition(self.a,replace(self.b,observation=dict(self.b.observation,truth_pose=[0,0,0])))
    def test_clock(self):
        for t in [-.2,.4,float('nan')]:
            with self.assertRaises(InfrastructureError): validate_transition(self.a,replace(self.b,sim_time=t))
    def test_action(self):
        self.assertEqual(decode_action([0,0]),(.2,0.))
        self.assertAlmostEqual(decode_action([-1,1])[0],-.2)
        self.assertAlmostEqual(decode_action([2,-2])[1],-.8)
        with self.assertRaises(ValueError):decode_action([float('nan'),0])
    def test_reject_wrong_action_shape(self):
        for action in (1, [1], [[1],[0]]):
            with self.assertRaises(ValueError): decode_action(action)
    def test_lidar_invalid_is_not_free(self):
        self.assertEqual(encode_lidar([1.,float('inf'),float('nan'),-1],sectors=1),[.1,.25,.25])
