import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'exploration'))
from aggressive import AggressiveMap, utility

class AggressiveTests(unittest.TestCase):
    def test_regional_pages_do_not_keep_returning_first_unsafe_batch(self):
        from dataclasses import replace
        data=np.zeros((180,180),dtype=np.int16);data[:,120:]=-1
        m=AggressiveMap(data,.1,(0,0,0))
        m.settings=replace(m.settings,maximum_candidates=2)
        first,info=m.candidates([4,8,0],regional=True)
        second,_=m.candidates([4,8,0],regional=True,candidate_offset=2)
        self.assertEqual(len(first),2);self.assertEqual(len(second),2)
        self.assertFalse({(c['x'],c['y']) for c in first}&{(c['x'],c['y']) for c in second})
        self.assertGreater(info['oriented_candidates_before_cap'],4)
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

    def test_alternate_heading_preserves_a_safe_observation_position(self):
        from unittest.mock import patch
        data=np.zeros((160,160),dtype=np.int16);data[:,120:]=-1
        data[:5,:]=100;data[-5:,:]=100;data[:,:5]=100
        model=AggressiveMap(data,.1,(0,0,0),sensor_offset=(-.7,0))
        original=model.is_safe
        # Simulate a heading-specific extra constraint while retaining every
        # real footprint/free-space check for the alternatives.
        with patch.object(model,'is_safe',side_effect=lambda x,y,a=0:abs(a)>.1 and original(x,y,a)):
            candidates,info=model.candidates([5,8,0],regional=True)
        self.assertTrue(candidates)
        self.assertTrue(all(original(c['x'],c['y'],c['yaw']) for c in candidates))
        self.assertTrue(all(abs(c['yaw'])>.1 for c in candidates))
        self.assertEqual(sum(info['pose_filter_counts'].values()),info['safe_cells'])

if __name__=='__main__':unittest.main()
