from dataclasses import replace
from pathlib import Path
import tempfile
import unittest
import numpy as np
from exploration_support.grid import Grid, capture_patch
from exploration_support.memory import Event, EventStore
from exploration_support.policy import Candidate, Config, filter_for_recheck, low_gain_cycle, parse_choice


class MemoryTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.path=Path(self.tmp.name)/"memory.sqlite3"
        self.store=EventStore(self.path)
        self.event=Event("task-1","m","epoch",1,(1.,1.,0.),"succeeded",True,gain_m2=.1)

    def tearDown(self): self.tmp.cleanup()

    def test_persistence(self):
        self.store.append(self.event)
        self.assertEqual(EventStore(self.path).events("m","epoch"),[self.event])

    def test_idempotent(self):
        self.assertTrue(self.store.append(self.event))
        self.assertFalse(self.store.append(self.event))
        self.assertEqual(len(self.store.events("m","epoch")),1)

    def test_conflict_rejected(self):
        self.store.append(self.event)
        with self.assertRaises(ValueError): self.store.append(replace(self.event,gain_m2=100.))

    def test_scope_isolation(self):
        self.store.append(self.event)
        self.assertEqual(self.store.events("other","epoch"),[])
        self.assertEqual(self.store.events("m","new-epoch"),[])

    def test_order(self):
        self.store.append(replace(self.event,event_id="task-2",step=2))
        self.store.append(self.event)
        self.assertEqual([e.step for e in self.store.events("m","epoch")],[1,2])

    @unittest.skipUnless(Path("/proc/self/fd").exists(),"Linux descriptor check")
    def test_1200_reads_do_not_leak_descriptors(self):
        self.store.append(self.event)
        before=len(list(Path("/proc/self/fd").iterdir()))
        for _ in range(1200): self.store.events("m","epoch")
        after=len(list(Path("/proc/self/fd").iterdir()))
        self.assertLessEqual(after,before+2)

    def test_approach_failure_needs_key(self):
        with self.assertRaises(ValueError): replace(self.event,failure_scope="approach")

    def test_nonfinite_gain_rejected(self):
        with self.assertRaises(ValueError): replace(self.event,gain_m2=float("nan"))


class PolicyTests(unittest.TestCase):
    def setUp(self):
        self.grid=Grid(np.zeros((40,40),dtype=int),.2)
        self.patch=capture_patch(self.grid,(2.,2.),.5,.2)
        self.e=Event("task-1","m","epoch",1,(2.,2.,0.),"succeeded",True,
                     gain_m2=.01,patch=self.patch)
        self.c=Candidate("new-id",2.1,2.,0.,10.,3.,"approach-west")

    def advice(self,events,c=None,grid=None,step=3):
        return filter_for_recheck(grid or self.grid,[c or self.c],events,"m","epoch",step)[0]

    def test_new_candidate_id_does_not_reset_visit(self):
        a=self.advice([self.e])
        self.assertFalse(a.retain_for_safety_recheck)
        self.assertEqual(a.reason,"RECENT_LOW_GAIN_OBSERVATION")

    def test_transit_is_not_banned_by_visit(self):
        self.assertTrue(self.advice([self.e],replace(self.c,purpose="transit")).retain_for_safety_recheck)

    def test_return_is_not_banned_by_visit(self):
        self.assertTrue(self.advice([self.e],replace(self.c,purpose="return")).retain_for_safety_recheck)

    def test_unknown_gain_does_not_count_as_zero(self):
        self.assertTrue(self.advice([replace(self.e,gain_m2=None)]).retain_for_safety_recheck)

    def test_large_gain_allows_revisit(self):
        self.assertTrue(self.advice([replace(self.e,gain_m2=2.)]).retain_for_safety_recheck)

    def test_no_patch_no_spatial_ban(self):
        self.assertTrue(self.advice([replace(self.e,patch=None)]).retain_for_safety_recheck)

    def test_wrong_epoch_not_reused(self):
        self.assertTrue(self.advice([replace(self.e,map_epoch="other")]).retain_for_safety_recheck)

    def test_goal_failure_scoped_to_pose(self):
        e=replace(self.e,status="rejected",failure_scope="goal_pose",reason="GOAL_FOOTPRINT_OCCUPIED")
        self.assertEqual(self.advice([e]).reason,"SAME_FAILED_GOAL_POSE")
        self.assertTrue(self.advice([e],replace(self.c,x=4.)).retain_for_safety_recheck)

    def test_approach_failure_does_not_blacklist_goal(self):
        e=replace(self.e,status="rejected",failure_scope="approach",approach_key="approach-west")
        self.assertFalse(self.advice([e]).retain_for_safety_recheck)
        self.assertTrue(self.advice([e],replace(self.c,approach_key="approach-east")).retain_for_safety_recheck)

    def test_no_automatic_time_release_of_failed_approach(self):
        e=replace(self.e,status="rejected",failure_scope="approach",approach_key="approach-west")
        self.assertFalse(self.advice([e],step=500).retain_for_safety_recheck)

    def test_local_change_only_permits_recheck(self):
        e=replace(self.e,status="rejected",failure_scope="goal_pose")
        a=self.grid.data.copy();a[8:13,8:13]=100
        advice=self.advice([e],grid=Grid(a,.2))
        self.assertTrue(advice.retain_for_safety_recheck)
        self.assertEqual(advice.reason,"RETAIN_FOR_EXISTING_SAFETY_CHECKS")

    def test_system_failure_is_not_geometric_obstacle(self):
        e=replace(self.e,status="indeterminate",failure_scope="system",stopped=False,gain_m2=None)
        self.assertTrue(self.advice([e]).retain_for_safety_recheck)
        # Local supervisor must STILL inhibit motion for an unresolved task.

    def cycle(self):
        poses=[(2.,2.,0.),(4.,2.,0.)]*2
        return [replace(self.e,event_id=f"e{i}",step=i,target=p,
                        patch=capture_patch(self.grid,p[:2],.5,.2)) for i,p in enumerate(poses,1)]

    def test_abab_low_gain_cycle(self):
        self.assertTrue(low_gain_cycle(self.cycle(),Config()))

    def test_aba_not_enough(self):
        self.assertFalse(low_gain_cycle(self.cycle()[:3],Config()))

    def test_abab_high_gain_not_loop(self):
        e=self.cycle();e[-1]=replace(e[-1],gain_m2=3.)
        self.assertFalse(low_gain_cycle(e,Config()))

    def test_abab_transit_not_loop(self):
        e=self.cycle();e[2]=replace(e[2],purpose="transit")
        self.assertFalse(low_gain_cycle(e,Config()))

    def test_abab_unconfirmed_not_loop(self):
        e=self.cycle();e[2]=replace(e[2],stopped=False)
        self.assertFalse(low_gain_cycle(e,Config()))

    def test_abab_different_epochs_not_loop(self):
        e=self.cycle();e[2]=replace(e[2],map_epoch="new")
        self.assertFalse(low_gain_cycle(e,Config()))

    def test_alternative_view_not_suppressed(self):
        self.assertTrue(self.advice(self.cycle(),replace(self.c,x=6.),step=5).retain_for_safety_recheck)

    def test_old_abab_still_blocks_repeated_goal(self):
        self.assertEqual(self.advice(self.cycle(),step=20).reason,"LOW_GAIN_ABAB_REPEAT")

    def test_extra_coordinates_rejected(self):
        with self.assertRaises(ValueError): parse_choice({"action":"move","frontier_id":"F","x":1},{"F"})

    def test_invalid_frontier_rejected(self):
        with self.assertRaises(ValueError): parse_choice({"action":"move","frontier_id":"OLD"},{"F"})

    def test_empty_set_accepts_stop(self):
        self.assertIsNone(parse_choice({"action":"stop","frontier_id":None},set()))

    def test_valid_selection(self):
        self.assertEqual(parse_choice({"action":"move","frontier_id":"F"},{"F"}),"F")

    def test_invalid_config(self):
        with self.assertRaises(ValueError): Config(same_place_m=-1.)
