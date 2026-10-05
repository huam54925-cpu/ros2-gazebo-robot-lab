import math,sys,unittest
from pathlib import Path
import numpy as np
W=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(W),str(W/'navigation_base/exploration')]
from robot_skills.operator_assist_geometry import sweep

class OperatorGeometryTests(unittest.TestCase):
    def grid(self):return {'data':np.zeros((220,220),dtype=np.int16),'resolution':.05,'origin':[0,0,0]}
    def test_reverse_preserves_actual_body_heading(self):
        result=sweep(self.grid(),[5,5,.3],distance=-1.)
        self.assertTrue(result['safe']);self.assertAlmostEqual(result['end_pose'][2],.3)
        self.assertAlmostEqual(result['end_pose'][0],5-math.cos(.3))
    def test_reverse_cannot_cross_unknown_body_cells(self):
        g=self.grid();g['data'][90:110,40:60]=-1
        self.assertFalse(sweep(g,[5,5,0],distance=-1.)['safe'])
    def test_offset_sensor_rotation_is_checked(self):
        g=self.grid();g['data'][100,137]=100
        self.assertFalse(sweep(g,[5,5,0],turn=math.pi)['safe'])
    def test_mixed_commands_and_unbounded_inputs_rejected(self):
        for kw in ({'distance':1,'turn':1},{'distance':7},{'turn':4},{'distance':float('nan')}):
            with self.assertRaises(ValueError):sweep(self.grid(),[5,5,0],**kw)
if __name__=='__main__':unittest.main()
