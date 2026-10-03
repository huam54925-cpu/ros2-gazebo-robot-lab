"""Map-only tests; no ROS publishers or running simulator required."""
import sys
from pathlib import Path
import math
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'exploration'))
from frontier import FrontierMap, Settings, distance_field


class FrontierTests(unittest.TestCase):
    def test_unknown_map_has_no_goals(self):
        m=FrontierMap(np.full((80,80),-1),.2,(0,0,0))
        goals,_=m.candidates((0,0));self.assertEqual(goals,[])

    def test_enclosed_known_room_has_no_frontier(self):
        data=np.zeros((80,80),dtype=int);data[[0,-1],:]=100;data[:,[0,-1]]=100
        m=FrontierMap(data,.2,(0,0,0));self.assertFalse(m.frontier.any())

    def test_partial_blocks_and_edges_do_not_wrap(self):
        data=np.full((9,9),100);data[:,0]=0;data[:,-1]=-1
        m=FrontierMap(data,.2,(0,0,0))
        # Left boundary itself is unknown outside, but right unknown cannot wrap into interior.
        data=np.full((20,20),100);data[5:15,1]=0;data[5:15,-1]=-1
        m=FrontierMap(data,.2,(0,0,0));self.assertFalse(m.frontier[5:15,1].any())
        m=FrontierMap(np.zeros((9,9)),.05,(0,0,0));self.assertTrue(m.unknown[-1,-1])

    def test_rotated_origin_and_goal_exclusion(self):
        data=np.full((100,100),-1);data[5:85,5:85]=0
        m=FrontierMap(data,.2,(-4,3,math.pi/2))
        xy=m.world(np.array([[30,20]]))[0]
        self.assertEqual(m.cell(*xy),(30,20))
        goals,_=m.candidates(m.world(np.array([[45,45]]))[0]);self.assertTrue(goals)
        for g in goals:self.assertTrue(m.is_safe(g['x'],g['y']))
        excluded=[(g['x'],g['y']) for g in goals]
        other,_=m.candidates(m.world(np.array([[45,45]]))[0],excluded)
        for g in other:
            self.assertTrue(all(math.hypot(g['x']-x,g['y']-y)>=m.settings.visited_radius for x,y in excluded))

    def test_clearance_never_overstates_point_distance(self):
        mask=np.zeros((50,50),dtype=bool);mask[20,20]=True
        d=distance_field(mask,.2);yy,xx=np.indices(mask.shape)
        exact=np.hypot(yy-20,xx-20)*.2
        self.assertTrue(np.all(d<=exact+1e-9))


if __name__=='__main__':unittest.main()
