"""Acquisition-to-current-pose scan transform tests."""
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scan_motion import OdomHistory,compensated_points


class ScanMotionTests(unittest.TestCase):
    def scan(self):
        return NS(header=NS(stamp=NS(sec=0,nanosec=0)),time_increment=0.,
                  ranges=[1.]+[math.inf]*359,range_max=10.)

    def test_translation_and_turn_apply_to_old_returns(self):
        for pose,expected in (([.2,0,0],[.8,0]),([0,0,math.pi/2],[0,-1])):
            h=OdomHistory();h.add(0.,[0,0,0],'vehicle/odom');h.add(.2,pose,'vehicle/odom')
            with patch('footprint_guard.scan_points',return_value=np.array([[1.,0.]])):
                points,evidence=compensated_points(self.scan(),h,.2)
            np.testing.assert_allclose(points[0],expected,atol=1e-9)
            self.assertEqual(evidence['remaining_pose_age_sim_s'],0.)

    def test_missing_pose_gap_and_clock_reset_fail_closed(self):
        h=OdomHistory();h.add(1.,[0,0,0],'vehicle/odom');h.add(2.,[0,0,0],'vehicle/odom')
        with self.assertRaisesRegex(ValueError,'unavailable'):h.at(.5)
        with self.assertRaisesRegex(ValueError,'gap'):h.at(1.5)
        with self.assertRaisesRegex(ValueError,'clock_reset'):h.add(.5,[0,0,0],'vehicle/odom')
        self.assertFalse(h.rows)


if __name__=='__main__':unittest.main()
