import math
from pathlib import Path
import sys
import unittest
sys.path[:0]=[str(Path(__file__).resolve().parents[2]),str(Path(__file__).resolve().parents[1]/'exploration')]
from observation import action_time_estimate, navigation_prediction
from robot_skills.timing import PhaseTimer


class TimingTests(unittest.TestCase):
    def test_reversed_subcell_connector_is_timing_only(self):
        poses=navigation_prediction([[-.02,0,0],[1,0,0]],[0,0,0],[1,0,0])
        original=[p[:] for p in poses]
        t=action_time_estimate(1.02,poses,.05)
        self.assertGreater(t['raw_turn_rad'],6)
        self.assertAlmostEqual(t['timing_turn_rad'],0)
        self.assertEqual(poses,original)
        self.assertGreater(t['budget_sim_s'],t['expected_sim_s'])

    def test_real_reversal_and_rotation_are_preserved(self):
        p=navigation_prediction([[-.2,0,0],[1,0,0]],[0,0,0],[1,0,0])
        t=action_time_estimate(1.4,p,.05)
        self.assertEqual(t['raw_turn_rad'],t['timing_turn_rad'])
        t=action_time_estimate(0,[[0,0,0],[0,0,math.pi/2]],.05)
        self.assertAlmostEqual(t['components']['turning_sim_s'],math.pi/.4)

    def test_phase_times_keep_clocks_separate(self):
        sim=[10.];wall=[100.]
        timer=PhaseTimer(lambda:sim[0],lambda:wall[0]);timer.mark('planning')
        sim[0]+=2;wall[0]+=8;timer.mark('navigation')
        sim[0]+=3;wall[0]+=12;timer.mark(None)
        self.assertEqual([r['sim_s'] for r in timer.rows],[2,3])
        self.assertEqual([r['wall_s'] for r in timer.rows],[8,12])
