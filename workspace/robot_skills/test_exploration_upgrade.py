import json
from pathlib import Path
import tempfile
import time
import unittest
import uuid
from unittest.mock import patch

from robot_skills import mission
from robot_skills.investigation import DEFAULT_HANDOFF
from robot_skills.store import Store
from robot_skills.exploration_upgrade import history_revision, memory_store, publish_terminal, reconcile, model_view


class UpgradeLedgerTests(unittest.TestCase):
    def setUp(self):
        temp=tempfile.TemporaryDirectory();self.addCleanup(temp.cleanup);self.store=Store(temp.name)
        state={'status':'available','sources':{'clock':{'fresh':True,'sim_time_s':1},
               'odometry':{'fresh':True,'velocity':{'x':0,'z':0}}}}
        with patch('navigation_base.safety_profile.FOOTPRINT_MODE',False):
            self.m=mission.start(self.store,mission.limits(),state,exploration_mode='memory')
        self.mid=self.m['mission_id']
        self.m=mission.update(self.store,self.mid,two_stage=True,phase='classical',handoff_policy=dict(DEFAULT_HANDOFF))
        self.catalog()

    def catalog(self,mode='memory',mid=None):
        self.store.set_meta('catalog',{'expires_unix_s':time.time()+120,'candidates':[
            {'frontier_id':'F_a','x':4.,'y':4.,'yaw':0.,'map_version':'map1','region_epoch':'epoch'},
            {'frontier_id':'F_b','x':5.,'y':4.,'yaw':0.,'map_version':'map1','region_epoch':'epoch'}],
            'exploration_upgrade':{'mode':mode,'mission_id':mid or self.mid,
                                  'history_revision':history_revision(self.store.tasks(),self.mid)}})

    def submit(self,rid=None,frontier='F_a'):
        return self.store.submit('execute_frontier',{'frontier_id':frontier},rid or str(uuid.uuid4()),self.mid)

    def finish(self,task,usable=False):
        self.store.update(task['task_id'],status='succeeded',stopped=True,reason='goal_reached',
            result={'after':{'pose':[4.1,4.1,.1]},'map_gain':{'usable_for_trend':usable,'observed_new_known_area_m2':.02},
                    'exploration_evidence':{'failure_scope':'none','patch':None}})

    def test_terminal_replay_records_actual_pose_once_and_never_reexecutes(self):
        rid=str(uuid.uuid4());t,launch=self.submit(rid);self.assertTrue(launch);self.finish(t)
        self.assertTrue(publish_terminal(self.store,t['task_id']))
        self.assertFalse(publish_terminal(self.store,t['task_id']))
        self.store.set_meta('stop_latched',True)
        replay,launch=self.submit(rid);self.assertFalse(launch)
        self.assertTrue(replay['replayed_without_execution'])
        events=memory_store(self.store).events(self.mid,'epoch')
        self.assertEqual(len(events),1);self.assertEqual(events[0].target,(4.1,4.1,.1))
        self.assertIsNone(events[0].gain_m2)
        self.assertTrue(self.store.meta('stop_latched'))

    def test_pending_terminal_and_changed_history_both_block_new_admission(self):
        t,_=self.submit();self.finish(t)
        catalog=self.store.meta('catalog');catalog['expires_unix_s']=time.time()+120
        self.store.set_meta('catalog',catalog) # isolate the history gate from normal TTL invalidation
        newer,launch=self.submit(frontier='F_b');self.assertFalse(launch)
        self.assertEqual(newer['reason'],'exploration_memory_pending')
        reconcile(self.store,self.mid)
        newer,launch=self.submit(frontier='F_b');self.assertFalse(launch)
        self.assertEqual(newer['reason'],'exploration_history_changed')
        self.catalog();newer,launch=self.submit(frontier='F_b')
        # Rejected attempts still consume the existing max-steps budget.
        self.assertFalse(launch);self.assertEqual(newer['reason'],'max_failures')

    def test_missing_actual_pose_cannot_be_a_completed_low_gain_observation(self):
        task,_=self.submit()
        self.store.update(task['task_id'],status='succeeded',stopped=True,reason='goal_reached',
                          result={'map_gain':{'usable_for_trend':True,'observed_new_known_area_m2':0.}})
        publish_terminal(self.store,task['task_id'])
        event=memory_store(self.store).events(self.mid,'epoch')[0]
        self.assertIsNone(event.gain_m2)
        self.assertEqual(event.purpose,'wait')

    def test_auxiliary_database_failure_does_not_rewrite_authoritative_terminal(self):
        t,_=self.submit();self.finish(t)
        with patch('robot_skills.exploration_upgrade.EventStore.append',side_effect=OSError('disk')):
            with self.assertRaises(OSError):publish_terminal(self.store,t['task_id'])
        task=self.store.get(t['task_id'])
        self.assertEqual(task['status'],'succeeded');self.assertTrue(task['stopped'])
        self.assertTrue(task['exploration_event_pending'])

    def test_same_request_conflict_precedes_new_mode_filters(self):
        rid=str(uuid.uuid4());self.submit(rid)
        with self.assertRaisesRegex(ValueError,'request_id_conflict'):self.submit(rid,'F_b')

    def test_wrong_mode_or_mission_catalog_is_not_admitted(self):
        self.catalog('shadow');t,launch=self.submit()
        self.assertFalse(launch);self.assertEqual(t['reason'],'exploration_mode_mismatch')
        self.catalog(mid=str(uuid.uuid4()));t,launch=self.submit()
        self.assertFalse(launch);self.assertEqual(t['reason'],'exploration_mode_mismatch')

    def test_stop_still_has_priority_over_pending_memory(self):
        t,_=self.submit();self.finish(t)
        stop,launch=self.store.submit('stop_robot',{},str(uuid.uuid4()))
        self.assertTrue(launch)
        newer,launch=self.submit(frontier='F_b');self.assertFalse(launch)
        self.assertEqual(newer['reason'],'stop_latched')

    def test_two_stage_budget_cannot_be_extended_in_place(self):
        with self.assertRaisesRegex(ValueError,'two_stage_budget_is_fixed_for_the_run'):
            mission.extend_operator_budget(self.store,self.mid,mission.limits(max_steps=4),2)

    def test_context_read_does_not_create_memory_database(self):
        from robot_skills.api import RobotSkills
        path=self.store.directory/'exploration-memory/events.sqlite3'
        self.assertFalse(path.exists())
        with patch('robot_skills.api.get_robot_status',return_value={'sources':{}}):
            RobotSkills(self.store,self.mid).get_decision_context()
        self.assertFalse(path.exists())

    def test_model_context_does_not_expose_other_mission_task_details(self):
        from robot_skills.api import RobotSkills
        tasks=[{'task_id':'old','mission_id':'old-map-session','status':'succeeded'},
               {'task_id':'current','mission_id':self.mid,'status':'succeeded'}]
        with patch('robot_skills.api.get_robot_status',return_value={'sources':{}}), \
             patch.object(self.store,'tasks',return_value=tasks):
            result=RobotSkills(self.store,self.mid).get_decision_context()
        self.assertEqual([t['task_id'] for t in result['recent_tasks']],['current'])

    def test_continuation_only_receives_history_from_current_map_session(self):
        from robot_skills.api import RobotSkills
        catalog=self.store.meta('catalog');catalog['region_epoch']='fresh';self.store.set_meta('catalog',catalog)
        tasks=[{'task_id':epoch,'mission_id':'prior','status':'aborted','reason':'body_sweep_nonfree',
                'candidate':{'region_epoch':epoch,'x':2.,'y':3.,'yaw':0.,'frontier_id':'F'},
                'result':{'private_old_map_data':'not_for_model'}} for epoch in ('old','fresh')]
        with patch('robot_skills.api.get_robot_status',return_value={'sources':{}}), \
             patch.object(self.store,'tasks',return_value=tasks):
            result=RobotSkills(self.store,self.mid).get_decision_context()
        self.assertEqual([t['task_id'] for t in result['map_session_task_history']],['fresh'])
        self.assertNotIn('private_old_map_data',json.dumps(result))

    def test_wide_search_is_opt_in_and_cannot_reuse_old_catalog(self):
        self.assertEqual(self.m['candidate_search'],'baseline')
        mission.update(self.store,self.mid,candidate_search='wide_4_5')
        task,launch=self.submit()
        self.assertFalse(launch)
        self.assertEqual(task['reason'],'candidate_search_mismatch')
        self.catalog()
        catalog=self.store.meta('catalog');catalog['candidate_search']='wide_4_5'
        self.store.set_meta('catalog',catalog)
        task,launch=self.submit(frontier='F_b')
        self.assertTrue(launch)
        self.assertEqual(task['candidate_search'],'wide_4_5')

    def test_operator_assistance_blocks_motion_but_allows_stop(self):
        self.catalog();self.store.set_meta('active_operator_assistance','local-operator')
        task,launch=self.submit(frontier='F_b')
        self.assertFalse(launch);self.assertEqual(task['reason'],'operator_assistance_active')
        task,launch=self.store.submit('stop_robot',{},str(uuid.uuid4()))
        self.assertTrue(launch);self.assertTrue(self.store.meta('stop_latched'))

    def test_catalog_with_different_clearance_cannot_dispatch(self):
        self.catalog()
        catalog=self.store.meta('catalog');catalog['path_clearance_m']=1.48
        self.store.set_meta('catalog',catalog)
        task,launch=self.submit(frontier='F_b')
        self.assertFalse(launch)
        self.assertEqual(task['reason'],'path_clearance_mismatch')

    def test_short_start_is_only_for_first_attempt_in_own_mission(self):
        from robot_skills.exploration_upgrade import short_start_for
        mission.update(self.store,self.mid,candidate_search='wide_4_5',short_start=True)
        self.assertTrue(short_start_for(self.store))
        task={'task_id':'first','mission_id':self.mid,'kind':'execute_frontier','status':'running'}
        with patch.object(self.store,'tasks',return_value=[task]):
            self.assertFalse(short_start_for(self.store))
            self.assertTrue(short_start_for(self.store,task))
        with patch.object(self.store,'tasks',return_value=[{**task,'status':'rejected'}]):
            self.assertFalse(short_start_for(self.store))
        with patch.object(self.store,'tasks',return_value=[{**task,'mission_id':'different'}]):
            self.assertTrue(short_start_for(self.store))
        mission.update(self.store,self.mid,short_start=False)
        self.assertFalse(short_start_for(self.store))

    @patch('navigation_base.safety_profile.FOOTPRINT_MODE',False)
    def test_wide_search_requires_memory_and_three_task_cap(self):
        for kwargs,reason in [({'candidate_search':'anything'},'invalid_candidate_search'),
                              ({'candidate_search':'wide_4_5'},'requires_memory'),
                              ({'candidate_search':'wide_4_5','exploration_mode':'memory'},'at_most_three')]:
            with self.assertRaisesRegex(ValueError,reason):
                mission.start(self.store,mission.limits(max_steps=4),{},**kwargs)

    def test_shadow_diagnostics_cannot_leak_through_action_tuples(self):
        value={'available_actions':{'F':('MOVE',{'x':1,'classical_score':2,'exploration_upgrade':{'policy':'deny'}})},
               'context':{'mission':{'exploration_mode':'shadow'},'exploration_upgrade':{'cycle':True}}}
        cleaned=model_view(value);encoded=json.dumps(cleaned)
        self.assertNotIn('exploration_upgrade',encoded);self.assertNotIn('shadow',encoded)
        self.assertEqual(cleaned['available_actions']['F'][1],{'x':1,'classical_score':2})

    def test_scan_statistics_keep_inf_nan_and_max_range_separate(self):
        from robot_skills.scan_diagnostics import summarize_scan
        result=summarize_scan([float('inf'),float('-inf'),float('nan'),12.,2.,.01,13.],.1,12.)
        self.assertEqual(result['counts'],{'finite_in_range':2,'finite_out_of_range':2,
                                          'positive_inf':1,'negative_inf':1,'nan':1,'at_max_range':1})
        self.assertEqual(result['map_cells_written'],0)
        self.assertEqual(result['inf_clearing_semantics'],'not_inferred')


if __name__=='__main__':unittest.main()
