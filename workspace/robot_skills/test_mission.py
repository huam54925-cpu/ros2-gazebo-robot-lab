import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_skills import mission
from robot_skills.store import Store
import tempfile,time,unittest,uuid

class MissionTests(unittest.TestCase):
    def setUp(self):
        tmp=tempfile.TemporaryDirectory(); self.addCleanup(tmp.cleanup); self.store=Store(tmp.name)
        self.state={'status':'available','sources':{'clock':{'fresh':True,'sim_time_s':100},
                    'odometry':{'fresh':True,'velocity':{'x':0,'z':0}}}}
        self.m=mission.start(self.store,mission.limits(),self.state)
    def test_limits_cannot_be_infinite_or_unbounded(self):
        for kw in ({'max_steps':6},{'max_steps':0},{'max_distance':float('nan')},{'max_wall_time':float('inf')}):
            with self.assertRaises(ValueError): mission.limits(**kw)
    def test_extended_budget_requires_explicit_local_profile_and_still_has_caps(self):
        budget=mission.limits(20,80,900,3600,3,profile='extended')
        self.assertEqual(mission.limits(**budget),budget)
        for kw in ({'max_steps':21},{'max_distance':81},{'max_sim_time':901},{'max_wall_time':3601}):
            with self.assertRaises(ValueError): mission.limits(profile='extended',**kw)
        with self.assertRaises(ValueError): mission.limits(profile='unlimited')
    def test_operator_extension_preserves_used_clocks_and_failure_bound(self):
        b=mission.limits(20,80,900,3600,2,profile='extended')
        changed=mission.extend_operator_budget(self.store,self.m['mission_id'],b,110)
        self.assertEqual(changed['started_monotonic_s'],self.m['started_monotonic_s'])
        self.assertEqual(changed['start_sim_s'],100)
        self.assertEqual(changed['deadline_monotonic_s'],self.m['started_monotonic_s']+3600)
        self.assertEqual(changed['budget_changes'][0]['previous_limits'],self.m['limits'])
        with self.assertRaisesRegex(ValueError,'failure_limit'):
            mission.extend_operator_budget(self.store,self.m['mission_id'],{**b,'max_failures':3},110)
    def test_operator_extension_cannot_revive_expired_or_stopped_mission(self):
        b=mission.limits(20,80,900,3600,2,profile='extended')
        with self.assertRaisesRegex(ValueError,'ending_mission'):
            mission.extend_operator_budget(self.store,self.m['mission_id'],b,221)
        self.store.set_meta('stop_latched',True)
        with self.assertRaisesRegex(ValueError,'stop_latched'):
            mission.extend_operator_budget(self.store,self.m['mission_id'],b,110)
    def test_mission_reserves_execution_against_other_clients(self):
        t,launch=self.store.submit('fixed_step',{},str(uuid.uuid4()))
        self.assertFalse(launch); self.assertEqual(t['reason'],'another_mission_active')
        t,launch=self.store.submit('fixed_step',{},str(uuid.uuid4()),self.m['mission_id'])
        self.assertTrue(launch)
    def test_wall_sim_distance_heartbeat_and_failure_budgets(self):
        m=self.m
        self.assertEqual(mission.reason(m,[],now=m['deadline_monotonic_s']),'max_wall_time')
        self.assertEqual(mission.reason(m,[],sim_time=220),'max_sim_time')
        self.assertEqual(mission.reason(m,[],sim_time=0),'simulation_clock_reset')
        self.assertEqual(mission.reason(m,[],now=m['heartbeat_monotonic_s']+16),'client_heartbeat_lost')
        task={'mission_id':m['mission_id'],'kind':'execute_frontier','status':'running','distance_odom_m':6}
        self.assertEqual(mission.reason(m,[task]),'max_distance')
        task.update(status='aborted',distance_odom_m=0)
        self.assertEqual(mission.reason(m,[task,task]),'max_failures')
    def test_max_steps_waits_for_last_terminal_and_blocks_next(self):
        mid=self.m['mission_id']; tasks=[{'mission_id':mid,'kind':'execute_frontier','status':'succeeded'}]*2
        tasks.append({'mission_id':mid,'kind':'execute_frontier','status':'running'})
        self.assertIsNone(mission.reason(self.m,tasks))
        tasks[-1]['status']='succeeded'
        self.assertEqual(mission.reason(self.m,tasks),'max_steps')
    def test_late_heartbeat_cannot_erase_stop_reason(self):
        mid=self.m['mission_id']
        mission.update(self.store,mid,status='stopping',stop_reason='operator_stop')
        m=mission.update(self.store,mid,status='running',heartbeat_monotonic_s=time.monotonic(),stop_reason='loop_completed')
        self.assertEqual(m['status'],'stopping'); self.assertEqual(m['stop_reason'],'operator_stop')
    def test_no_auto_resume(self):
        self.store.set_meta('stop_latched',True)
        mission.update(self.store,self.m['mission_id'],status='finished',stopped=True)
        self.assertTrue(self.store.meta('stop_latched'))
        with self.assertRaisesRegex(ValueError,'stop_latched'):
            mission.start(self.store,mission.limits(),self.state)

if __name__=='__main__': unittest.main()
