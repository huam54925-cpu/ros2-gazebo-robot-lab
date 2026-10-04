"""Offline integration of advisory memory with recorded-map geometry."""
from dataclasses import asdict
import math
from pathlib import Path
import tempfile
import sys
import unittest
import numpy as np

sys.path[:0]=[str(Path(__file__).resolve().parents[2]),str(Path(__file__).resolve().parents[1]/'exploration')]
from robot_skills.store import Store
from robot_skills.upgrade_geometry import UpgradeGeometry, failure_evidence, footprint_evidence, witness_key
from exploration_support.grid import Grid,capture_patch
from exploration_support.memory import Event


class UpgradeGeometryTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);self.store=Store(temp.name)
        self.data=np.zeros((100,100),dtype=np.int16);self.data[:,80:]=-1
        self.snapshot={'data':self.data,'resolution':.25,'origin':(0,0,0),'frame':'map','stamp_sim_s':1.}
        self.grid=Grid(self.data,.25)
        self.bridge=UpgradeGeometry(self.store,'mission','epoch',self.snapshot,[3.,4.,0.],(-.7,0.,0.),12.,'memory')
        self.bridge.step=5
        self.c={'frontier_id':'new-id','x':8.,'y':4.,'yaw':0.,'estimated_gain_m2':2.,'planned_length_m':5.,'classical_score':.5}
        self.path=[[3.,4.,0.],[8.,4.,0.]]

    def failure(self):
        return Event('terminal:test','mission','epoch',1,(8.,4.,0.),'aborted',True,
                     reason='path_guard_clearance',failure_scope='approach',approach_key=witness_key([5.,4.,0.]),
                     patch=capture_patch(self.grid,(5.,4.),3.5,.25))

    def test_failed_approach_preserves_other_entry_to_same_goal(self):
        self.bridge.events=[self.failure()]
        self.assertFalse(self.bridge.advise(self.c,self.path)['retain_for_safety_recheck'])
        detour=[[3.,4.,0.],[3.,8.,math.pi/2],[8.,8.,0.],[8.,4.,-math.pi/2]]
        self.assertTrue(self.bridge.advise(self.c,detour)['retain_for_safety_recheck'])

    def test_distant_map_change_does_not_release_failed_approach(self):
        self.bridge.events=[self.failure()]
        changed=self.data.copy();changed[60:70,60:70]=100
        self.bridge.grid=Grid(changed,.25)
        self.assertFalse(self.bridge.advise(self.c,self.path)['retain_for_safety_recheck'])

    def test_relevant_change_only_grants_recheck_not_safety(self):
        self.bridge.events=[self.failure()]
        changed=self.data.copy();changed[10:24,14:28]=100
        self.bridge.grid=Grid(changed,.25)
        self.assertTrue(self.bridge.advise(self.c,self.path)['retain_for_safety_recheck'])
        from path_safety import PathSafety
        check=PathSafety(changed,.25,(0,0,0),'changed').evaluate(self.path,self.path[0],self.path[-1],(0,0))
        self.assertFalse(check['safe'])
        self.assertFalse(self.bridge.summary()['is_motion_permission'])

    def test_same_observation_new_id_is_suppressed_but_transit_is_retained(self):
        self.bridge.events=[Event('terminal:low','mission','epoch',1,(8.,4.,0.),'succeeded',True,gain_m2=.02,
                                  patch=capture_patch(self.grid,(8.,4.),8.,.25))]
        self.assertFalse(self.bridge.advise(self.c,self.path)['retain_for_safety_recheck'])
        self.assertTrue(self.bridge.advise({**self.c,'transit':True},self.path)['retain_for_safety_recheck'])

    def test_incomparable_gain_is_not_a_zero_gain_ban(self):
        self.bridge.events=[Event('terminal:unknown','mission','epoch',1,(8.,4.,0.),'succeeded',True,gain_m2=None,
                                  patch=capture_patch(self.grid,(8.,4.),8.,.25))]
        self.assertTrue(self.bridge.advise(self.c,self.path)['retain_for_safety_recheck'])

    def test_abab_diagnostic_requires_unchanged_local_evidence(self):
        for step,pose in enumerate([(8.,4.,0.),(10.,4.,0.)]*2,1):
            self.bridge.events.append(Event('terminal:'+str(step),'mission','epoch',step,pose,
                'succeeded',True,gain_m2=.02,patch=capture_patch(self.grid,pose[:2],8.,.25)))
        self.assertTrue(self.bridge.summary()['low_gain_cycle_detected'])
        # Exceed the explicit 5% local-patch semantic-change threshold.
        changed=self.data.copy();changed[8:32,22:46]=100
        self.bridge.grid=Grid(changed,.25)
        summary=self.bridge.summary()
        self.assertTrue(summary['low_gain_pattern_detected'])
        self.assertFalse(summary['low_gain_cycle_detected'])

    def test_proxy_zero_keeps_original_id_and_score_and_is_not_a_rejection(self):
        row,retain=self.bridge.enrich(self.c,self.path)
        self.assertTrue(retain)
        for key,value in self.c.items():self.assertEqual(row[key],value)
        self.assertEqual(row['exploration_upgrade']['visibility']['incremental_boundary_proxy_m2'],0)
        self.assertFalse(row['exploration_upgrade']['visibility']['visibility_is_safety_certificate'])

    def test_preflight_rejections_do_not_count_as_completed_observations(self):
        outcome={'status':4,'clearance':{'safe':False,'reason':'path_guard_clearance','worst_base_pose':[5.,4.,0.],
                                       'worst_sensor_xy':[4.3,4.]}}
        self.bridge.rejected_plan(self.c,outcome);self.bridge.rejected_plan(self.c,outcome)
        self.assertEqual(len(self.bridge.failures),1)
        self.assertEqual(self.bridge.summary()['events'],0)
        self.assertFalse(self.bridge.summary()['low_gain_cycle_detected'])
        self.assertFalse(self.bridge.advise(self.c,self.path)['retain_for_safety_recheck'])

    def test_no_path_or_timeout_is_not_an_invented_spatial_obstacle(self):
        for result,code in [({'status':'aborted','reason':'goal_timeout'},'GOAL_TIMEOUT'),
                            ({'status':'rejected','planning':{'status':6,'error_code':1}},'NO_NAV2_PATH')]:
            evidence=failure_evidence(self.grid,self.c,result)
            self.assertEqual(evidence['failure_scope'],'system');self.assertIsNone(evidence['patch'])
            self.assertEqual(evidence['reason_code'],code)

    def test_goal_footprint_reports_unknown_occupied_uncertain_separately(self):
        for value,code in [(-1,'GOAL_FOOTPRINT_UNKNOWN'),(100,'GOAL_FOOTPRINT_OCCUPIED'),(50,'GOAL_FOOTPRINT_UNCERTAIN')]:
            data=self.data.copy();data[14:18,14:18]=value
            evidence=footprint_evidence({**self.snapshot,'data':data},[4.,4.,0.])
            self.assertEqual(evidence['reason_code'],code)

    def test_archive_has_original_raw_map_and_no_motion_authority(self):
        row,_=self.bridge.enrich(self.c,self.path)
        self.bridge.save('offline',{},[self.c],{'candidates':[row]})
        path=self.store.directory/'exploration-upgrade/offline.map.npz'
        with np.load(path) as z:self.assertTrue(np.array_equal(z['data'],self.data))
        self.assertTrue(path.with_name('offline.json').exists())


if __name__=='__main__':unittest.main()
