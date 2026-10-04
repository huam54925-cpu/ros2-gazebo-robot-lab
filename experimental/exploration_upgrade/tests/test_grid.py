import math
import unittest
import numpy as np
from exploration_support.grid import (Cell, Grid, capture_patch, observed_delta,
    patch_change, sensor_pose, visible_unknown_boundary)
from exploration_support.adapter import enrich_existing_candidates


class GridTests(unittest.TestCase):
    def test_four_semantics(self):
        g = Grid(np.array([[-1,0,24,25,64,65,100]]), 1.)
        self.assertEqual(g.labels().tolist(), [[0,1,1,3,3,2,2]])

    def test_invalid_costmap_rejected(self):
        with self.assertRaises(ValueError): Grid(np.array([[255]], dtype=np.uint8), 1.)

    def test_float_data_rejected(self):
        with self.assertRaises(ValueError): Grid(np.array([[0.0]]), 1.)

    def test_bad_resolution_rejected(self):
        with self.assertRaises(ValueError): Grid(np.array([[0]]), 0.)

    def test_immutable_copy(self):
        a = np.array([[0]])
        g = Grid(a, 1.)
        a[0,0] = 100
        self.assertEqual(int(g.data[0,0]), 0)
        with self.assertRaises(ValueError): g.data[0,0] = 100

    def test_unknown_not_free(self):
        self.assertEqual(Grid(np.array([[-1]]), 1.).label_at(.5,.5), Cell.UNKNOWN)

    def test_frontier_only_free_unknown(self):
        g = Grid(np.array([[100,100,100],[0,-1,25],[100,100,100]]), 1.)
        self.assertEqual(int(g.frontier_mask().sum()), 1)
        self.assertTrue(g.frontier_mask()[1,0])

    def test_uncertain_does_not_create_frontier(self):
        self.assertFalse(Grid(np.array([[0,50]]), 1.).frontier_mask().any())

    def test_no_wraparound(self):
        g = Grid(np.array([[0,100,-1],[100,100,100]]), 1.)
        self.assertFalse(g.frontier_mask()[0,0])

    def test_edge_frontier_explicit(self):
        g = Grid(np.zeros((3,3), dtype=int), 1.)
        self.assertEqual(int(g.frontier_mask().sum()), 0)
        self.assertEqual(int(g.frontier_mask(True).sum()), 8)

    def test_rotated_origin_roundtrip(self):
        g = Grid(np.zeros((8,8), dtype=int), .2, (1.,2.,math.pi/2))
        self.assertEqual(g.cell(*g.world(3,4)), (3,4))

    def test_outside_is_separate(self):
        g = Grid(np.array([[0]]), 1.)
        self.assertEqual(g.label_at(-.1,.5), Cell.OUTSIDE)

    def test_sensor_offset_rotates(self):
        x,y,a = sensor_pose((2.,3.,math.pi/2),(-.7,0.,0.))
        self.assertAlmostEqual(x,2.)
        self.assertAlmostEqual(y,2.3)

    def test_wall_blocks_unknown(self):
        a = np.full((11,11), -1, dtype=int)
        a[3:8,3:8] = 100
        a[4:7,4:7] = 0
        v = visible_unknown_boundary(Grid(a,1.),(5.5,5.5,0.),8.,360)
        self.assertEqual(v.proxy_m2, 0.)
        self.assertEqual(v.rays_hit_occupied,360)

    def test_opening_reveals_unknown(self):
        a = np.full((11,11),-1,dtype=int)
        a[3:8,3:8] = 100
        a[4:7,4:7] = 0
        a[5,7] = 0
        self.assertGreater(visible_unknown_boundary(Grid(a,1.),(5.5,5.5,0.),8.,360).proxy_m2,0.)

    def test_unknown_is_not_transparent(self):
        a = np.full((9,9),-1,dtype=int)
        a[4,4] = 0
        v = visible_unknown_boundary(Grid(a,1.),(4.5,4.5,0.),8.,360)
        self.assertLessEqual(len(v.first_unknown_cells),8)
        self.assertTrue(all(max(abs(r-4),abs(c-4)) == 1 for r,c in v.first_unknown_cells))

    def test_uncertain_stops_ray(self):
        a = np.full((9,9),-1,dtype=int)
        a[3:6,3:6] = 50
        a[4,4] = 0
        v = visible_unknown_boundary(Grid(a,1.),(4.5,4.5,0.),8.,360)
        self.assertEqual(v.proxy_m2,0.)
        self.assertEqual(v.rays_hit_uncertain,360)

    def test_outside_not_counted_as_area(self):
        v = visible_unknown_boundary(Grid(np.zeros((3,3),dtype=int),1.),(1.5,1.5,0.),8.)
        self.assertEqual(v.proxy_m2,0.)
        self.assertGreater(v.rays_leave_map,0)

    def test_visibility_nonfree_origin_rejected(self):
        with self.assertRaises(ValueError):
            visible_unknown_boundary(Grid(np.array([[-1]]),1.),(.5,.5,0.))

    def test_first_unknown_cells_deduplicated(self):
        a = np.full((9,9),-1,dtype=int); a[4,4]=0
        g=Grid(a,1.)
        v1=visible_unknown_boundary(g,(4.5,4.5,0.),8.,180)
        v2=visible_unknown_boundary(g,(4.5,4.5,0.),8.,360)
        self.assertEqual(v1.proxy_m2,v2.proxy_m2)

    def test_enrichment_preserves_candidate_id_and_score(self):
        a=np.zeros((15,15),dtype=int);a[:,12:]=-1
        c={"id":"F_batch_3","x":8.5,"y":7.5,"yaw":0.,"estimated_gain_m2":99.}
        r=enrich_existing_candidates(Grid(a,1.),(3.5,7.5,0.),[c],(0.,0.,0.))[0]
        self.assertEqual(r["id"],c["id"])
        self.assertEqual(r["estimated_gain_m2"],99.)
        self.assertFalse(r["visibility_is_safety_certificate"])
        self.assertNotIn("incremental_boundary_proxy_m2",c)


class PatchAndMetricTests(unittest.TestCase):
    def test_same_world_patch_after_padding(self):
        a=np.zeros((10,10),dtype=int);a[4,4]=100
        g=Grid(a,.2)
        p=capture_patch(g,(1.,1.),.3,.1)
        b=np.pad(a,((2,0),(2,0)),constant_values=-1)
        moved=Grid(b,.2,(-.4,-.4,0.))
        self.assertFalse(patch_change(moved,p)[0])

    def test_padding_unknown_is_not_new_local_evidence(self):
        a=np.zeros((5,5),dtype=int)
        g=Grid(a,1.)
        p=capture_patch(g,(4.5,4.5),2.,.5)
        b=np.pad(a,((0,4),(0,4)),constant_values=-1)
        self.assertFalse(patch_change(Grid(b,1.),p)[0])

    def test_remote_change_does_not_unlock(self):
        a=np.zeros((40,40),dtype=int)
        g=Grid(a,.2);p=capture_patch(g,(1.,1.),.3,.1)
        a[30,30]=100
        self.assertFalse(patch_change(Grid(a,.2),p)[0])

    def test_local_change_enables_recheck(self):
        a=np.zeros((40,40),dtype=int)
        g=Grid(a,.2);p=capture_patch(g,(1.,1.),.3,.1)
        a[3:8,3:8]=100
        self.assertTrue(patch_change(Grid(a,.2),p)[0])

    def test_gain_not_array_size(self):
        a=np.zeros((3,3),dtype=int)
        d=observed_delta(Grid(a,1.),Grid(np.pad(a,1,constant_values=-1),1.,(-1.,-1.,0.)),True)
        self.assertEqual(d.newly_observed_m2,0.)

    def test_gain_unknown_to_known(self):
        a=np.array([[0,-1],[-1,-1]])
        b=np.array([[0,100],[0,-1]])
        d=observed_delta(Grid(a,.5),Grid(b,.5),True)
        self.assertEqual(d.newly_observed_m2,.5)
        self.assertEqual(d.lost_observed_m2,0.)

    def test_lost_information_separate(self):
        a=np.array([[0,-1]]); b=np.array([[-1,100]])
        d=observed_delta(Grid(a,1.),Grid(b,1.),True)
        self.assertEqual((d.newly_observed_m2,d.lost_observed_m2,d.net_known_area_m2),(1.,1.,0.))

    def test_epoch_change_not_comparable(self):
        g=Grid(np.zeros((2,2),dtype=int),1.)
        with self.assertRaises(ValueError): observed_delta(g,g,False)

    def test_subcell_shift_not_comparable(self):
        a=np.zeros((2,2),dtype=int)
        with self.assertRaises(ValueError): observed_delta(Grid(a,1.),Grid(a,1.,(.1,0.,0.)),True)

    def test_confidence_update_not_new_observation(self):
        self.assertEqual(observed_delta(Grid(np.array([[50]]),1.),Grid(np.array([[0]]),1.),True).newly_observed_m2,0.)

    def test_patch_wrong_frame_fails(self):
        g=Grid(np.zeros((10,10),dtype=int),1.)
        p=capture_patch(g,(4.,4.))
        with self.assertRaises(ValueError): patch_change(Grid(g.data,1.,frame="odom"),p)
