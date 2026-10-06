import math,sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from footprint_guard import check_sweep,scan_points

class FootprintGuardTests(unittest.TestCase):
    def test_straight_gap_ten_cm_passes_five_cm_stops(self):
        for gap,safe in [(.10,True),(.05,False)]:
            points=np.array([[x,sign*(.96+gap)] for x in np.linspace(-2,2,60) for sign in (-1,1)])
            self.assertEqual(check_sweep(points,(.10,0))['safe'],safe)
    def test_collision_and_braking_forward_reverse(self):
        self.assertFalse(check_sweep([[.3,0]],(0,0))['safe'])
        self.assertFalse(check_sweep([[.44,0]],(.10,0),(.10,0),.3)['safe'])
        self.assertFalse(check_sweep([[-1.85,0]],(-.10,0),(-.10,0),.3)['safe'])
        self.assertTrue(check_sweep([[2.,0]],(.10,0))['safe'])
    def test_forward_alarm_allows_checked_reverse_but_never_contact(self):
        self.assertFalse(check_sweep([[.44,0]],(.04,0),scan_age=.5)['safe'])
        self.assertTrue(check_sweep([[.44,0]],(-.04,0),scan_age=.5)['safe'])
        for direction in (-.04,.04):
            self.assertFalse(check_sweep([[.38,0]],(direction,0),scan_age=.5)['safe'])
    def test_turn_sweeps_tail_outside_straight_envelope(self):
        point=[[-1.6,-.75]]
        self.assertTrue(check_sweep(point,(.1,0))['safe'])
        self.assertFalse(check_sweep(point,(0,.12),scan_age=.4)['safe'])
    def test_measured_motion_checked_even_when_requested_zero(self):
        self.assertFalse(check_sweep([[.44,0]],(0,0),(.1,0),.3)['safe'])
    def test_scan_contract_and_offset(self):
        scan=NS(header=NS(frame_id='vehicle/lidar'),ranges=[math.inf]*360,
                range_min=.1,range_max=12.,angle_min=-math.pi,angle_increment=2*math.pi/359)
        scan.ranges[0]=1.
        np.testing.assert_allclose(scan_points(scan)[0],[-1.7057095,0],atol=1e-8)
        scan.ranges[2]=math.nan
        with self.assertRaises(ValueError):scan_points(scan)
    def test_invalid_motion(self):
        self.assertFalse(check_sweep([], (math.nan,0))['safe'])

if __name__=='__main__':unittest.main()
