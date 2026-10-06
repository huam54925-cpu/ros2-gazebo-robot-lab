"""Regression cases for future execution; deliberately NOT run in this edit."""
import math
from pathlib import Path
import sys
import time
import unittest
import uuid
from unittest.mock import patch, Mock
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'navigation_base'),str(ROOT/'navigation_base/exploration')]
from robot_skills.recovery import (POLICY, eligible, straight_retreat, straight_sweep,
                                   source_reason, reserve, update_attempt, probe_reason)
from robot_skills.test_investigation import InvestigationFixture
from robot_skills.investigation import task_progress


def trace():
    return [{'pose':[i*.01,0.,0.],'sim_s':1+i*.1,'frame':'vehicle/odom'} for i in range(31)]


class GeometryTests(unittest.TestCase):
    def test_reverse_retraces_only_recent_straight_forward_suffix(self):
        self.assertAlmostEqual(straight_retreat(trace(),[.3,0.,0.],4.),.20)
        curved=trace()
        for row in curved[:-3]:row['pose'][2]=math.pi/4
        with self.assertRaisesRegex(ValueError,'no_recent_straight'):
            straight_retreat(curved,[.3,0,0],4.)

    def test_gap_reset_age_and_backward_history_do_not_invent_a_route(self):
        with self.assertRaisesRegex(ValueError,'stale'):straight_retreat(trace(),[.3,0,0],5.)
        rows=trace();rows[-2]['sim_s']=3.
        with self.assertRaisesRegex(ValueError,'no_recent_straight'):straight_retreat(rows,[.3,0,0],4.)
        rows=trace()
        for row in rows:row['pose'][0]=.3-row['pose'][0]
        with self.assertRaisesRegex(ValueError,'no_recent_straight'):straight_retreat(rows,[0,0,0],4.)

    def test_stationary_worker_boundary_preserves_timestamps_and_rejects_moving_gap(self):
        rows=trace()+[{'pose':[.3,0,0],'sim_s':8.,'frame':'vehicle/odom'}]
        with self.assertRaisesRegex(ValueError,'no_recent_straight'):straight_retreat(rows,[.3,0,0],8.)
        self.assertAlmostEqual(straight_retreat(rows,[.3,0,0],8.,(4.,8.)),.20)
        rows[-1]['pose']=[.4,0,0]
        with self.assertRaisesRegex(ValueError,'no_recent_straight'):
            straight_retreat(rows,[.4,0,0],8.,(4.,8.))

    def test_reverse_sweep_keeps_heading_and_rejects_unknown_behind_rear(self):
        import numpy as np
        grid={'data':np.zeros((120,120),dtype=int),'resolution':.1,'origin':[0,0,0]}
        poses=[]
        def body(*args):poses.append(args[3]);return {'safe':True,'reason':'clear'}
        with patch('navigation_base.exploration.observation.body_check',side_effect=body):
            self.assertTrue(straight_sweep(grid,[5,5,0],-.2)['safe'])
        self.assertTrue(all(p[2]==0 for p in poses))
        self.assertAlmostEqual(poses[-1][0],4.8)
        # Rear edge moves from 3.27 to 3.07: unknown cell is outside initial
        # padded body but inside the reverse sweep.
        grid['data'][49:51,30:31]=-1
        self.assertTrue(straight_sweep(grid,[5,5,0],0)['safe'])
        self.assertFalse(straight_sweep(grid,[5,5,0],-.2)['safe'])

    def test_feedback_stop_and_generic_errors_never_authorize_reverse(self):
        self.assertTrue(eligible('Failed to make progress'))
        self.assertTrue(eligible('scan_body_sweep'))
        for why in ('stop_requested','stop_unconfirmed','odom_at_stale','localization_stale',
                    'goal_timeout','nav2_aborted','config_contract_mismatch','max_distance'):
            self.assertFalse(eligible(why))

    def test_short_motion_success_is_not_an_exploration_goal(self):
        for kind in ('probe_forward','recover_short_reverse'):
            p=task_progress({'kind':kind,'status':'succeeded','stopped':True,
                             'result':{'map_gain':{'usable_for_trend':True,'observed_new_known_area_m2':0}}})
            self.assertFalse(p['goal_reached']);self.assertEqual(p['observed_gain_m2'],0)


class RecoveryStoreTests(InvestigationFixture, unittest.TestCase):
    def running_probe(self):
        iid=self.store.meta('mission:'+self.mid).get('active_investigation')
        if not iid:iid=self.create()['investigation_id']
        payload={'investigation_id':iid,'map_epoch':'epoch'}
        task,launch=self.store.submit('probe_forward',payload,str(uuid.uuid4()),self.mid)
        self.assertTrue(launch)
        return self.store.update(task['task_id'],status='running'),payload

    def finish_probe(self, task):
        return self.store.update(task['task_id'],status='aborted',reason='short_motion_no_progress',
            stopped=True,dispatched=True,result={'map_epoch':'epoch','odom_trace':trace(),
                'before':{'pose':[.3,0,0]},'after':{'pose':[.3,0,0]}})

    def test_recovery_attempt_is_durable_and_cannot_be_replayed(self):
        task,_=self.running_probe();row=reserve(self.store,task,[0,0,0],'epoch',task['task_id'])
        update_attempt(self.store,self.mid,row['attempt'],status='denied',reason='body_sweep_nonfree')
        with self.assertRaisesRegex(RuntimeError,'already_attempted'):
            reserve(self.store,task,[3,0,0],'epoch',task['task_id'])
        self.assertEqual(self.store.meta('recovery_attempts:'+self.mid)[0]['status'],'denied')

    def test_location_limit_persists_across_tasks_and_stop_blocks_reservation(self):
        for i in range(POLICY['max_recoveries_per_location']):
            task,_=self.running_probe();reserve(self.store,task,[i*.1,0,0],'epoch',task['task_id'])
            self.finish_probe(task)
        task,_=self.running_probe()
        with self.assertRaisesRegex(RuntimeError,'at_location'):reserve(self.store,task,[0,0,0],'epoch',task['task_id'])
        self.store.set_meta('stop_latched',True)
        with self.assertRaisesRegex(RuntimeError,'stop_requested'):reserve(self.store,task,[9,0,0],'epoch',task['task_id'])

    def test_explicit_recovery_requires_owned_stopped_latest_failure(self):
        task,payload=self.running_probe();source=self.finish_probe(task)
        self.assertIsNone(source_reason(source,self.store.tasks(),self.mid,'epoch',payload['investigation_id']))
        self.assertIsNotNone(source_reason({**source,'stopped':False},[source],self.mid,'epoch',payload['investigation_id']))
        self.assertIsNotNone(source_reason(source,[source],'other','epoch',payload['investigation_id']))
        newer,_=self.running_probe()
        self.assertEqual(source_reason(source,self.store.tasks(),self.mid,'epoch',payload['investigation_id']),
                         'recovery_source_superseded')

    def test_short_skill_request_replay_and_no_speed_override(self):
        task,payload=self.running_probe()
        replay,launch=self.store.submit('probe_forward',payload,task['request_id'],self.mid)
        self.assertFalse(launch);self.assertEqual(replay['task_id'],task['task_id'])
        with self.assertRaisesRegex(ValueError,'invalid_short_motion_payload'):
            self.store.submit('probe_forward',{**payload,'speed':.4},str(uuid.uuid4()),self.mid)
        source=self.finish_probe(task)
        self.assertEqual(probe_reason([source],self.mid,[.3,0,0],'epoch'),'probe_approach_cooldown')
        self.assertIsNone(probe_reason([source],self.mid,[.3,0,1.],'epoch'))


class RecoveryCancellationTests(unittest.TestCase):
    def test_unconfirmed_stop_never_reaches_reverse_reservation(self):
        from robot_skills.recovery_worker import RecoveryMixin
        worker=SimpleNamespace(two_stage=True,dispatched=True,task={'kind':'navigate_to_pose'},
                               handle=object(),pending_goal=None,cancel_active=Mock(return_value=False),
                               retreat=Mock(),health=Mock(),canceling=False)
        with self.assertRaisesRegex(RuntimeError,'stop_unconfirmed'):
            RecoveryMixin.automatic_retreat(worker,{'reason':'no_motion_progress'})
        worker.retreat.assert_not_called()
        self.assertFalse(worker.canceling)

    def test_pending_acceptance_timeout_is_not_assumed_stopped(self):
        from robot_skills.recovery_worker import RecoveryMixin
        worker=SimpleNamespace(two_stage=True,dispatched=True,task={'kind':'navigate_to_pose'},
                               handle=None,pending_goal=object(),wait=Mock(side_effect=RuntimeError('action_response_timeout')),
                               cancel_active=Mock(),retreat=Mock(),health=Mock(),canceling=False)
        with self.assertRaisesRegex(RuntimeError,'action_response_timeout'):
            RecoveryMixin.automatic_retreat(worker,{'reason':'no_motion_progress'})
        worker.retreat.assert_not_called()
        worker.cancel_active.assert_not_called()
        self.assertIsNotNone(worker.pending_goal)


if __name__=='__main__':unittest.main()
