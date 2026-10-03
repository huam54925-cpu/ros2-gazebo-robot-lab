import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'exploration'))
from aggressive import AggressiveMap, utility

class AggressiveTests(unittest.TestCase):
    def test_oriented_body_does_not_enter_unknown(self):
        data=np.zeros((120,120),dtype=np.int16);data[:,60:]=-1
        m=AggressiveMap(data,.1,(0,0,0))
        self.assertTrue(m.is_safe(5.1,6.,0.))
        self.assertFalse(m.is_safe(5.1,6.,np.pi))
        self.assertFalse(m.is_safe(6.1,6.,0.))

    def test_known_wall_occludes_unknown_gain(self):
        data=np.zeros((160,160),dtype=np.int16);data[:,100:]=-1
        open_map=AggressiveMap(data,.1,(0,0,0))
        data[:,95:100]=100
        blocked_map=AggressiveMap(data,.1,(0,0,0))
        self.assertGreater(open_map.gain(5.,8.),0.)
        self.assertEqual(blocked_map.gain(5.,8.),0.)

    def test_utility_prefers_information_but_penalizes_cost(self):
        near={'dx':1.,'dy':0.,'estimated_gain_m2':2.}
        far={'dx':5.,'dy':0.,'estimated_gain_m2':10.}
        self.assertGreater(utility(far,5.,0),utility(near,1.,0))
        self.assertGreater(utility(far,5.,0),utility(far,10.,0))
        self.assertGreater(utility(far,5.,0),utility(far,5.,np.pi))

if __name__=='__main__':unittest.main()
