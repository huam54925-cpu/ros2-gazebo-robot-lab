"""Analytic cell-square clearances: clearance recovery must not miss contact."""
import math,sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'exploration'))
from observation import polygon_square_distances,body_check,body_safe

class BodyPrecisionTests(unittest.TestCase):
    def test_exact_distances_axis_corner_overlap_and_rotation(self):
        body=np.array([[-1.,-1],[1,-1],[1,1],[-1,1]])
        centers=np.array([[1.3,0],[1.3,1.4],[0,0],[1.05,0]])
        np.testing.assert_allclose(polygon_square_distances(body,centers,.2),[.2,math.hypot(.2,.3),0,0],atol=1e-12)
        angle=.37;c,s=math.cos(angle),math.sin(angle)
        rotated=body@np.array([[c,s],[-s,c]])
        # A square wholly enclosed in the body must still collide at any yaw.
        self.assertEqual(polygon_square_distances(rotated,np.array([[0.,0.]]),.1)[0],0.)
    def test_unknown_square_outside_margin_is_not_falsely_rejected(self):
        data=np.zeros((80,80),dtype=int);data[40,51]=-1
        body=np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]])
        # Body x max .5; cell x interval [.55,.60]: exact gap .05.
        # Config padding .01 + motion bound .025 = .035, so it is clear.
        result=body_check(data,.05,(-2,-2,0),[0,0,0],body,.01)
        self.assertTrue(result['safe'])
        # Old margin .01 + full cell diagonal + .025 wrongly covered it.
        self.assertLess(.575-.5,.01+.05*math.sqrt(2)+.025)
        self.assertFalse(body_safe(data,.05,(-2,-2,0),[.025,0,0],body,.01))
    def test_unknown_occupied_uncertain_and_edge_contacts_all_block(self):
        body=np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]])
        for value,key in [(-1,'unknown'),(100,'occupied'),(40,'uncertain')]:
            data=np.zeros((80,80),dtype=int);data[40,50]=value
            result=body_check(data,.05,(-2,-2,0),[0,0,0],body,.05)
            self.assertFalse(result['safe']);self.assertEqual(result['blocking_counts'][key],1)
        self.assertFalse(body_safe(data,.05,(-2,-2,0),[1.5,0,0],body,.05))
    def test_rotated_map_frame_preserves_same_physical_test(self):
        data=np.zeros((80,80),dtype=int);data[40,51]=-1
        body=np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]])
        for theta in [0,.4,-1.2]:
            c,s=math.cos(theta),math.sin(theta)
            # Map local pose (2.025,2), transformed to world with origin (5,6).
            pose=[5+c*2.025-s*2,6+s*2.025+c*2,theta]
            self.assertFalse(body_safe(data,.05,(5,6,theta),pose,body,.01))

if __name__=='__main__':unittest.main()
