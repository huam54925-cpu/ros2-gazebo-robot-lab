"""Read-only wider-search witnesses, without a ROS domain or robot commands."""
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path[:0]=[str(Path(__file__).resolve().parents[2]),str(Path(__file__).resolve().parents[1]/'exploration')]
from aggressive import AggressiveMap
from observation import body_safe,navigation_footprint,body_sweep
from robot_skills.compare_viewpoints import visible_search,visible_unknown_boundary as fast_visible
from exploration_support.grid import Grid,visible_unknown_boundary as reference_visible


class ViewpointComparisonTests(unittest.TestCase):
    def test_fast_ray_implementation_matches_reference_with_all_cell_classes(self):
        data=np.zeros((60,60),dtype=np.int16)
        data[:,45:]=-1;data[10:20,32]=100;data[35:45,30]=50
        grid=Grid(data,.2,(-5,-5,.1))
        for pose in [(0,0,0),(1,0,.7),(-1,1,-2.1)]:
            self.assertEqual(fast_visible(grid,pose,8.,180),reference_visible(grid,pose,8.,180))

    def fixture(self):
        data=np.zeros((80,80),dtype=np.int16);data[:,60:]=-1
        model=AggressiveMap(data,.25,(0,0,0),sensor_offset=(-.7057095,0),sensor_range_m=8.)
        # One known-safe sampled base position, ~6.75m from the boundary.
        model.safe[:]=False;model.safe[40,32]=True
        return data,model

    def test_wider_visible_point_passes_same_body_check_without_map_mutation(self):
        data,model=self.fixture();before=data.copy();robot=[3.,10.,0.]
        original,info=model.candidates(robot)
        self.assertFalse(original);self.assertEqual(info['pose_filter_counts']['frontier_too_far'],1)
        candidates,diag=visible_search(model,robot,(-.7057095,0,0),8.,rays=90)
        self.assertTrue(candidates)
        c=candidates[0]
        self.assertGreater(c['frontier_distance_m'],3.)
        self.assertGreater(c['robust_incremental_boundary_cells'],0)
        self.assertTrue(model.is_safe(c['x'],c['y'],c['yaw']))
        self.assertEqual(model.settings.maximum_frontier_distance,3.)
        self.assertTrue(np.array_equal(data,before))
        self.assertIsNone(diag['actual_map_gain_m2'])
        self.assertFalse(diag['visibility_is_safety_certificate'])

    def test_wall_occludes_unknown_instead_of_counting_it_as_gain(self):
        data,model=self.fixture();data[:,56]=100
        model=AggressiveMap(data,.25,(0,0,0),sensor_offset=(-.7057095,0),sensor_range_m=8.)
        model.safe[:]=False;model.safe[40,32]=True
        candidates,diag=visible_search(model,[3.,10.,0.],(-.7057095,0,0),8.,rays=90)
        self.assertFalse(candidates)

    def test_safe_endpoints_do_not_bypass_unknown_under_intermediate_body(self):
        data=np.zeros((160,160),dtype=np.int16);data[80,80]=-1
        origin=(-10.,-10.,0.);body,padding=navigation_footprint()
        start=[-3.,0.,0.];goal=[3.,0.,0.]
        for p in (start,goal):self.assertTrue(body_safe(data,.125,origin,p,body,padding))
        result=body_sweep(data,.125,origin,[start,goal],start,goal)
        self.assertFalse(result['safe']);self.assertEqual(result['reason'],'body_sweep_nonfree')
