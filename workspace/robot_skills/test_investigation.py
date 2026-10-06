"""ROS-free regressions for persistent missions. Authored but not run in this change."""
import math
from pathlib import Path
import sys
import tempfile
import time
import unittest
import uuid
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_skills.store import Store
from robot_skills import investigation as inv, mission


class InvestigationTests(unittest.TestCase):
    def setUp(self):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup)
        self.store=Store(tmp.name);self.mid=str(uuid.uuid4());now=time.monotonic()
        self.m={'mission_id':self.mid,'status':'running','phase':'ai_investigation','two_stage':True,
                'limits':mission.limits(None,80,900,3600,12,profile='mission'),
                'start_sim_s':10,'deadline_monotonic_s':now+3600,'heartbeat_monotonic_s':now,
                'handoff_policy':dict(inv.DEFAULT_HANDOFF),'investigation_ids':[]}
        self.store.set_meta('mission:'+self.mid,self.m)
        self.store.set_meta('active_mission',self.mid)
        self.store.set_meta('regional_epoch',{'id':'epoch'})
        self.pose={'x':3.,'y':2.,'yaw':0.}

    def create(self):
        return inv.create(self.store,self.mid,self.pose,'Investigate occluded boundary',
                          'observe_structure',str(uuid.uuid4()))

    def submit(self,iid,request=None,poses=None):
        return self.store.submit('navigate_through_poses',
            {**inv.poses_payload(poses or [self.pose],'epoch'),'investigation_id':iid},
            request or str(uuid.uuid4()),self.mid)

    def test_step_count_does_not_end_whole_mission_and_queries_do_not_consume_failures(self):
        tasks=[{'mission_id':self.mid,'kind':'execute_frontier','status':'succeeded'} for _ in range(100)]
        tasks += [{'mission_id':self.mid,'kind':'plan_navigation','status':'aborted'} for _ in range(30)]
        self.assertIsNone(mission.reason(self.m,tasks,sim_time=11))
        self.assertEqual(mission.progress(self.m,tasks)['failures'],0)
        self.assertEqual(mission.reason(self.m,tasks,sim_time=910),'max_sim_time')

    def test_navigation_is_scoped_and_replay_does_not_dispatch_twice(self):
        investigation=self.create();request=str(uuid.uuid4())
        task,launch=self.submit(investigation['investigation_id'],request)
        self.assertTrue(launch)
        replay,launch=self.submit(investigation['investigation_id'],request)
        self.assertFalse(launch);self.assertEqual(replay['task_id'],task['task_id'])
        with self.assertRaisesRegex(ValueError,'conflict'):
            self.submit(investigation['investigation_id'],request,[{**self.pose,'x':5.}])

    def test_old_epoch_and_classical_phase_cannot_submit_coordinates(self):
        investigation=self.create()
        self.store.set_meta('regional_epoch',{'id':'new_epoch'})
        task,launch=self.submit(investigation['investigation_id'])
        self.assertFalse(launch);self.assertEqual(task['reason'],'map_epoch_changed')
        self.store.set_meta('regional_epoch',{'id':'epoch'})
        self.store.set_meta('mission:'+self.mid,{**self.m,'phase':'classical'})
        task,launch=self.submit(investigation['investigation_id'])
        self.assertFalse(launch);self.assertEqual(task['reason'],'active_investigation_required')

    def test_stop_is_not_released_by_investigation_tools(self):
        investigation=self.create();self.store.set_meta('stop_latched',True)
        task,launch=self.submit(investigation['investigation_id'])
        self.assertFalse(launch);self.assertEqual(task['reason'],'stop_latched')
        self.assertTrue(self.store.meta('stop_latched'))

    def test_terminal_failure_is_remembered_atomically_once(self):
        investigation=self.create();task,_=self.submit(investigation['investigation_id'])
        self.store.update(task['task_id'],status='aborted',reason='path_rejected',stopped=True,
                          result={'before':{'pose':[0,0,0]},'after':{'pose':[0,0,0]}})
        self.store.update(task['task_id'],status='aborted',reason='path_rejected')
        events=self.store.meta('investigation_memory:'+self.mid)
        self.assertEqual(len(events),1);self.assertEqual(events[0]['failure_code'],'NO_PATH')
        self.assertEqual(self.store.meta('investigation:'+investigation['investigation_id'])['task_ids'],[task['task_id']])
        self.assertEqual(inv.repetition_reason(events,[self.pose],'epoch',[0,0,0]),'failed_approach_cooldown')
        self.assertIsNone(inv.repetition_reason(events,[{**self.pose,'yaw':math.pi/2}],'epoch',[0,0,0]))
        self.assertIsNone(inv.repetition_reason(events,[self.pose],'epoch',[4,0,0]))

    def test_navigation_arrival_cannot_claim_passage_or_new_observations(self):
        investigation=self.create();task,_=self.submit(investigation['investigation_id'])
        self.store.update(task['task_id'],status='succeeded',reason='goal_reached',stopped=True,result={})
        for status in ('passage_verified','observations_collected','blocked'):
            with self.assertRaises(ValueError):
                inv.finish(self.store,self.mid,investigation['investigation_id'],status,'claim',[task['task_id']])
        final=inv.finish(self.store,self.mid,investigation['investigation_id'],'unresolved','No sufficient evidence',[task['task_id']])
        self.assertFalse(final['structure_confirmed'])

    def test_unknown_gain_does_not_trigger_oscillation(self):
        events=[{'map_epoch':'epoch','goal':self.pose,'progress':{'pose':[i%2,0,0],
                 'observed_gain_m2':None}} for i in range(4)]
        self.assertFalse(inv.oscillating(events))
        for e in events:e['progress']['observed_gain_m2']=0
        self.assertTrue(inv.oscillating(events))
        events[-1]['progress']['new_waypoint_reached']=True
        self.assertFalse(inv.oscillating(events))

    def test_handoff_requires_repeated_completed_search_not_timeout_or_repoll(self):
        self.store.set_meta('mission:'+self.mid,{**self.m,'phase':'classical'})
        snapshot={'stopped':True,'map_epoch':'epoch'}
        bad={'status':'catalog_timeout','search_complete':False}
        self.assertEqual(inv.transition(self.store,self.mid,bad,snapshot)['phase'],'classical')
        first={'catalog_id':'a','status':'no_safe_reachable_frontiers','search_complete':True,
               'more_candidates':False,'candidates':[]}
        self.assertEqual(inv.transition(self.store,self.mid,first,snapshot)['phase'],'classical')
        self.assertEqual(inv.transition(self.store,self.mid,first,snapshot)['phase'],'classical')
        second={**first,'catalog_id':'b'}
        handed=inv.transition(self.store,self.mid,second,snapshot)
        self.assertEqual(handed['phase'],'ai_investigation')
        self.assertFalse(handed['handoff']['world_complete'])

    def test_input_validation_rejects_nonfinite_and_missing_coordinates(self):
        for pose in ({'x':0,'y':0},{'x':float('nan'),'y':0,'yaw':0},{'x':True,'y':0,'yaw':0}):
            with self.assertRaises(ValueError):inv.finite_pose(pose)
        with self.assertRaises(ValueError):inv.poses_payload([], 'epoch')


if __name__=='__main__':unittest.main()
