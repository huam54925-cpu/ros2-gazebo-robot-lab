import sys, unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parents[1]/'exploration')]
from aggressive import AggressiveMap
from path_safety import PathSafety

class FootprintProfileTests(unittest.TestCase):
    def corridor(self,width=2.2):
        r=.05;data=np.zeros((200,240),dtype=int)
        y=(np.arange(200)+.5)*r-5
        data[abs(y)>=width/2]=100
        return data,r,(-6,-5,0)
    def test_prefilter_retains_body_legal_pose_and_transit(self):
        data,r,origin=self.corridor()
        with patch("safety_profile.FOOTPRINT_MODE",False):
            self.assertFalse(AggressiveMap(data,r,origin).is_safe(0,0,0))
        with patch('safety_profile.FOOTPRINT_MODE',True):
            model=AggressiveMap(data,r,origin)
            self.assertTrue(model.is_safe(0,0,0))
            self.assertTrue(model.safe[model.cell(0,0)])
            self.assertFalse(model.is_safe(0,0,1.57))
    def test_full_straight_pass_and_turn_fail(self):
        data,r,origin=self.corridor();model=PathSafety(data,r,origin,'test')
        with patch('safety_profile.FOOTPRINT_MODE',True):
            self.assertTrue(model.evaluate([[0,0,0],[2,0,0]],[0,0,0],[2,0,0],[-.7057095,0])['safe'])
            self.assertFalse(model.evaluate([[0,0,0],[0,0,1.57]],[0,0,0],[0,0,1.57],[-.7057095,0])['safe'])
    def test_subcell_start_connector_does_not_invent_reverse_turn(self):
        data,r,origin=self.corridor();model=PathSafety(data,r,origin,'connector')
        with patch('safety_profile.FOOTPRINT_MODE',True):
            short=model.evaluate([[-.01,0,3.14],[.1,0,0],[1,0,0]],[0,0,0],[1,0,0],[-.7057095,0])
            self.assertTrue(short['safe'])
            real_reverse=model.evaluate([[-.2,0,0],[.1,0,0],[1,0,0]],[0,0,0],[1,0,0],[-.7057095,0])
            self.assertFalse(real_reverse['safe'])
    def test_unknown_and_wall_still_reject(self):
        data,r,origin=self.corridor();data[100,120]=-1
        with patch('safety_profile.FOOTPRINT_MODE',True):
            result=PathSafety(data,r,origin,'test').evaluate([[0,0,0],[1,0,0]],[0,0,0],[1,0,0],[-.7057095,0])
            self.assertFalse(result['safe'])
            self.assertGreater(result['body_sweep']['cell_evidence']['blocking_counts']['unknown'],0)

if __name__=='__main__':unittest.main()
