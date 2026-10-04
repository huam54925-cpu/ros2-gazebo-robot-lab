"""Unknown semantics and viewpoint evidence must never grant motion authority."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'exploration'))
from observation import Visibility, map_semantics
from frontier import FrontierMap


class UnknownSpaceTests(unittest.TestCase):
    def test_intermediate_occupancy_is_not_unobserved_or_confirmed_wall(self):
        data=np.full((30,30),50);data[10:20,10:20]=0
        model=FrontierMap(data,.2,(0,0,0))
        self.assertFalse(model.unknown.any())
        self.assertFalse(model.occupied.any())
        self.assertFalse(model.frontier.any())
        self.assertTrue(model.uncertain[0,0])
        self.assertFalse(model.safe[0,0])

    def test_raw_classes_do_not_modify_the_map_or_invent_world_coverage(self):
        data=np.array([[-1,0,24,25,64,65,100]])
        original=data.copy();summary=map_semantics(data,.5)
        self.assertEqual(summary['cells'],{'free':2,'occupied':2,'unknown':1,'uncertain_occupancy':2})
        self.assertEqual(summary['area_m2']['unknown'],.25)
        self.assertTrue(np.array_equal(data,original))
        self.assertIsNone(summary['world_coverage'])

    def test_known_wall_unknown_is_explained_but_never_credited_as_visible(self):
        grid=np.zeros((100,140),dtype=int);grid[:,100:]=-1
        open_view=Visibility(grid,.2,(0,0,0),(0,0),12)
        self.assertGreater(open_view.analyze([15,10,0])['visible_unknown_area_proxy_m2'],0)
        grid[:,90:92]=100
        blocked=Visibility(grid,.2,(0,0,0),(0,0),12).analyze([15,10,0])
        self.assertEqual(blocked['unknown_type'],'UNKNOWN_OCCLUDED')
        self.assertEqual(blocked['visible_unknown_area_proxy_m2'],0)
        self.assertGreater(blocked['occluded_unknown_area_proxy_m2'],0)
        self.assertEqual(blocked['historical_unknown_cause'],'UNDETERMINED')

    def test_uncertain_barrier_is_not_claimed_to_be_a_confirmed_wall(self):
        grid=np.zeros((100,140),dtype=int);grid[:,100:]=-1;grid[:,90:92]=50
        view=Visibility(grid,.2,(0,0,0),(0,0),12).analyze([15,10,0])
        self.assertEqual(view['unknown_type'],'UNKNOWN_UNCERTAIN')
        self.assertEqual(view['visible_unknown_area_proxy_m2'],0)
        self.assertEqual(view['occluded_unknown_area_proxy_m2'],0)
        self.assertGreater(view['uncertain_unknown_area_proxy_m2'],0)

    def test_beyond_current_range_is_relative_geometry_not_a_historical_cause(self):
        grid=np.zeros((100,120),dtype=int);grid[:,60:]=-1
        view=Visibility(grid,.2,(0,0,0),(0,0),6).analyze([10,10,0],[1,10,0])
        self.assertEqual(view['unknown_type'],'UNKNOWN_OUT_OF_RANGE')
        self.assertGreater(view['beyond_current_sensor_range_proxy_m2'],0)
        self.assertEqual(view['historical_unknown_cause'],'UNDETERMINED')
        missing=Visibility(grid,.2,(0,0,0),(0,0)).analyze([10,10,0],[1,10,0])
        self.assertIsNone(missing['beyond_current_sensor_range_proxy_m2'])
        self.assertEqual(missing['unknown_type'],'UNKNOWN_OPEN')

    def test_actual_short_sensor_range_limits_the_proxy(self):
        grid=np.zeros((100,120),dtype=int);grid[:,60:]=-1
        view=Visibility(grid,.2,(0,0,0),(0,0),2).analyze([8,10,0])
        self.assertEqual(view['visible_unknown_area_proxy_m2'],0)
        self.assertEqual(view['proxy_range_m'],2)

    def test_sensor_offset_and_heading_change_geometry(self):
        grid=np.zeros((100,120),dtype=int);grid[:,60:]=-1
        view=Visibility(grid,.2,(0,0,0),(-1,0),3)
        self.assertEqual(view.analyze([10,10,0])['visible_unknown_area_proxy_m2'],0)
        self.assertGreater(view.analyze([10,10,np.pi])['visible_unknown_area_proxy_m2'],0)

    def test_unknown_depth_is_not_unlimited_visible_information(self):
        grid=np.zeros((100,120),dtype=int);grid[:,60:]=-1
        view=Visibility(grid,.2,(0,0,0),(0,0),8).analyze([10,10,0])
        self.assertGreater(view['visible_unknown_area_proxy_m2'],0)
        self.assertGreater(view['uncertain_unknown_area_proxy_m2'],0)
        self.assertFalse(view['unknown_is_traversable'])


if __name__=='__main__':unittest.main()
