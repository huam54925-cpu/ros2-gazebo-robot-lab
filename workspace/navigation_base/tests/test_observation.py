import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'exploration'))
from observation import (Visibility,rotation_path,body_safe,check_rotation,navigation_footprint,map_change)

class ObservationTests(unittest.TestCase):
    def test_zero_offset_full_lidar_has_no_yaw_gain(self):
        grid=np.full((80,80),-1); grid[25:55,25:55]=0
        v=Visibility(grid,.2,(-8,-8,0),(0,0))
        self.assertEqual(v.novel(rotation_path([0,0,0],math.pi,1),[0,0,0]),0)
    def test_known_wall_blocks_hidden_unknown(self):
        grid=np.zeros((80,80),dtype=int); grid[:,50]=100; grid[:,51:]=-1
        v=Visibility(grid,.2,(-8,-8,0),(0,0))
        self.assertEqual(v.visible([0,0,0]),set())
    def test_offset_can_change_visible_set(self):
        grid=np.full((80,80),-1); grid[25:55,25:55]=0
        v=Visibility(grid,.2,(-8,-8,0),(-.7,0))
        self.assertGreater(v.novel(rotation_path([0,0,0],math.pi/2,1),[0,0,0]),0)
    def test_signed_full_turn_is_not_collapsed(self):
        for angle in (2*math.pi,-2*math.pi):
            path=rotation_path([0,0,3.1],angle,2)
            self.assertAlmostEqual(path[-1][2]-path[0][2],angle)
            self.assertGreater(len(path),200)
    def test_safe_endpoints_do_not_hide_body_sweep_into_unknown(self):
        grid=np.zeros((200,200),dtype=int); grid[116,116]=-1
        body=np.array([[-2,-.1],[2,-.1],[2,.1],[-2,.1]])
        self.assertTrue(body_safe(grid,.05,(-5,-5,0),[0,0,0],body,0))
        self.assertTrue(body_safe(grid,.05,(-5,-5,0),[0,0,math.pi/2],body,0))
        check=check_rotation(grid,.05,(-5,-5,0),'v',[0,0,0],math.pi/2,(0,0),body,0)
        self.assertFalse(check['safe']); self.assertEqual(check['reason'],'body_sweep_nonfree')
    def test_sensor_middle_sweep_rejected(self):
        grid=np.zeros((240,240),dtype=int); grid[120,161]=100
        body=np.array([[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]])
        check=check_rotation(grid,.05,(-6,-6,0),'v',[0,0,-math.pi/2],math.pi,(.7,0),body,0)
        self.assertFalse(check['safe']); self.assertEqual(check['reason'],'path_guard_clearance')
    def test_footprint_comes_from_both_navigation_costmaps(self):
        body,padding=navigation_footprint()
        self.assertEqual(len(body),6); self.assertEqual(padding,.05)
    def test_map_expansion_and_integer_origin_translation(self):
        old={'data':np.zeros((2,2)), 'resolution':1,'origin':(0,0,0),'frame':'map'}
        new={'data':np.zeros((3,3)), 'resolution':1,'origin':(-1,-1,0),'frame':'map'}
        result=map_change(old,new)
        self.assertEqual(result['observed_new_known_area_m2'],5)
        self.assertEqual(result['observed_lost_known_area_m2'],0)
        self.assertEqual(result['gain_status'],'FEEDBACK_INCOMPLETE')
    def test_unaligned_grid_is_not_fake_gain(self):
        old={'data':np.zeros((2,2)), 'resolution':1,'origin':(0,0,0),'frame':'map'}
        self.assertEqual(map_change(old,{**old,'origin':(.25,0,0)})['gain_status'],'UNCOMPARABLE')
        self.assertEqual(map_change(old,{**old,'resolution':.5})['gain_status'],'UNCOMPARABLE')
    def test_identical_maps_do_not_prove_low_gain(self):
        grid={'data':np.zeros((2,2)), 'resolution':1,'origin':(0,0,0),'frame':'map'}
        result=map_change(grid,grid)
        self.assertEqual(result['gain_status'],'FEEDBACK_INCOMPLETE')
        self.assertEqual(result['net_known_area_change_m2'],0)

    def test_new_free_and_occupied_are_separate_from_reclassification(self):
        before={'data':np.array([[-1,-1,0,-1]]),'resolution':1,'origin':(0,0,0),'frame':'map'}
        after={**before,'data':np.array([[0,100,100,50]])}
        result=map_change(before,after)
        self.assertEqual(result['observed_new_free_area_m2'],1)
        self.assertEqual(result['observed_new_occupied_area_m2'],1)
        self.assertEqual(result['observed_new_ambiguous_area_m2'],1)
        self.assertEqual(result['observed_new_known_area_m2'],3)

    def test_fractional_origin_uses_exact_overlap_and_labels_boundary_artifacts(self):
        before={'data':np.zeros((2,2)),'resolution':1,'origin':(0,0,0),'frame':'map'}
        after={**before,'origin':(.25,.5,0)}
        result=map_change(before,after,allow_fractional=True)
        self.assertAlmostEqual(result['observed_new_known_area_m2'],4-1.75*1.5)
        self.assertAlmostEqual(result['observed_lost_known_area_m2'],4-1.75*1.5)
        self.assertAlmostEqual(result['net_known_area_change_m2'],0)
        self.assertEqual(result['registration'],'fractional_cell_area_overlap')
        self.assertEqual(result['gain_status'],'FEEDBACK_INCOMPLETE')

    def test_only_unknown_map_padding_does_not_add_area(self):
        before={'data':np.zeros((2,2)),'resolution':1,'origin':(0,0,0),'frame':'map'}
        data=np.full((4,4),-1);data[1:3,1:3]=0
        result=map_change(before,{**before,'data':data,'origin':(-1,-1,0)})
        self.assertEqual(result['observed_new_known_area_m2'],0)
        self.assertEqual(result['observed_lost_known_area_m2'],0)

    def test_known_cells_disappearing_are_reported_as_loss(self):
        before={'data':np.zeros((2,2)),'resolution':1,'origin':(0,0,0),'frame':'map'}
        result=map_change(before,{**before,'data':np.array([[0,0],[-1,-1]])})
        self.assertEqual(result['observed_new_known_area_m2'],0)
        self.assertEqual(result['observed_lost_known_area_m2'],2)

    def test_visibility_cache_does_not_mutate_current_view(self):
        grid=np.full((80,80),-1); grid[25:55,25:55]=0
        v=Visibility(grid,.2,(-8,-8,0),(-.7,0))
        original=v.visible([0,0,0])
        first=v.novel([[0,0,1]],[0,0,0],[[1,0,0]])
        second=v.novel([[0,0,1]],[0,0,0],[[1,0,0]])
        self.assertEqual(original,v.visible([0,0,0])); self.assertEqual(first,second)

if __name__=='__main__': unittest.main()
