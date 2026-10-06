"""Regression tests for the consolidated runtime."""
import math
from pathlib import Path
import sys
import time
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch, Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_skills.recovery import reverse_steps, POLICY
from robot_skills.recovery_worker import RecoveryMixin
from robot_skills.investigation import failure_evidence, finish, transition
from robot_skills.mission import classical_boundary, classical_remaining, action_budget
from robot_skills.test_investigation import InvestigationFixture


class ShortMotionTests(unittest.TestCase):
    def scan_worker(self):
        from navigation_base.scan_motion import OdomHistory
        h=OdomHistory();h.add(0.,[0,0,0],'vehicle/odom')
        scan=NS(header=NS(stamp=NS(sec=0,nanosec=100_000_000)),time_increment=0.,ranges=[1.],range_max=10.)
        worker=NS(latest={'raw_scan':scan},scan_history=h,store=Mock(),identifier='task',
                  node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=120_000_000))))
        worker.store.should_cancel.return_value=False
        return worker

    def test_scan_before_odometry_waits_for_real_bracket(self):
        import numpy as np
        worker=self.scan_worker()
        worker.spin_once=Mock(side_effect=lambda:worker.scan_history.add(.12,[.12,0,0],'vehicle/odom'))
        with patch('footprint_guard.scan_points',return_value=np.array([[1.,0.]])):
            points,meta=RecoveryMixin.synchronized_scan_points(worker)
        worker.spin_once.assert_called_once()
        np.testing.assert_allclose(points[0],[.98,0.])
        self.assertEqual(meta['compensated_to_odom_sim_s'],.12)

    def test_scan_sync_timeout_and_cancel_do_not_extrapolate(self):
        worker=self.scan_worker();worker.spin_once=Mock()
        with patch('robot_skills.recovery_worker.time.monotonic',side_effect=[0.,0.,6.]):
            with self.assertRaisesRegex(RuntimeError,'scan_pose_sync_timeout'):
                RecoveryMixin.synchronized_scan_points(worker)
        worker.store.should_cancel.return_value=True
        with self.assertRaisesRegex(RuntimeError,'stop_requested'):
            RecoveryMixin.synchronized_scan_points(worker)

    def test_reverse_steps_do_not_exceed_history_and_include_smaller_options(self):
        self.assertEqual([round(v,2) for v in reverse_steps(.2)],[.2,.15,.1,.05])
        self.assertEqual([round(v,2) for v in reverse_steps(.13)],[.13,.1,.05])
        self.assertEqual(reverse_steps(.04),[])

    def test_reverse_geometric_rejection_falls_back_before_single_dispatch(self):
        def clearance(distance):
            if abs(distance)>.1:raise RuntimeError('body_sweep_nonfree')
        worker=NS(store=object(),task={'mission_id':'mission'},identifier='task',distance=0.,
                  ensure_epoch=lambda:'epoch',pose=lambda:[0,0,0],health=lambda:None,
                  recovery_feedback=lambda:[0,0,0],short_clearance=clearance,
                  node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=0))),
                  short_action=Mock(return_value={'status':4}),observe=lambda _:None,
                  latest={'map_version':'new'},handle=None)
        result={}
        with patch('robot_skills.recovery_worker.reserve',return_value={'attempt':1}), \
             patch('robot_skills.recovery_worker.update_attempt'), \
             patch('robot_skills.recovery_worker.straight_retreat',return_value=.2):
            RecoveryMixin.retreat(worker,result,'source',[])
        self.assertEqual(worker.short_action.call_count,1)
        self.assertAlmostEqual(worker.short_action.call_args.args[0],-.1)
        self.assertEqual(len(result['recoveries'][0]['shorter_step_checks']),2)

    def test_slow_clock_is_progress_and_frozen_clock_is_failure(self):
        ctx={'odom_start':[0,0,0],'direction':1,'distance':.2,'distance_start':0.,
             'progress':0.,'last_sim_s':0.,'clock_advanced_wall':0.,'last_progress_sim_s':0.,
             'start_sim_s':0.,'allowance_sim_s':10.,'last_check_at':50.,'epoch':'epoch'}
        # 50 wall seconds, but only 5 simulation seconds; displacement is valid.
        worker=NS(short_motion_active=ctx,canceling=False,action_result=NS(done=lambda:False),
                  recovery_feedback=lambda:[.18,0,0],distance=.18,
                  node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=5_000_000_000))))
        with patch('robot_skills.recovery_worker.time.monotonic',return_value=50.):
            RecoveryMixin.monitor_short_motion(worker)
        self.assertEqual(ctx['last_progress_sim_s'],5.)
        with patch('robot_skills.recovery_worker.time.monotonic',return_value=61.):
            with self.assertRaisesRegex(RuntimeError,'clock_stalled'):RecoveryMixin.monitor_short_motion(worker)


class HandoffEvidenceTests(InvestigationFixture,unittest.TestCase):
    def test_reserve_handoff_keeps_total_budget_and_does_not_claim_saturation(self):
        m={**self.m,'phase':'classical','ai_reserve':{'max_distance':24.,'max_sim_time':270.,'max_wall_time':1080.,'max_failures':4}}
        self.store.set_meta('mission:'+self.mid,m)
        left=classical_remaining(m,[],sim_time=640)
        self.assertEqual(left['sim_s'],0)
        self.assertEqual(classical_boundary(m,[],640),'classical_budget_reserved')
        self.assertIn('classical_budget_reserved',action_budget(m,[],630,20,1)['reasons'])
        handed=transition(self.store,self.mid,{}, {'stopped':True,'map_epoch':'epoch'},'classical_budget_reserved')
        self.assertEqual(handed['phase'],'ai_investigation')
        self.assertEqual(handed['limits'],m['limits'])
        self.assertEqual(handed['handoff']['trigger'],'resource_reservation')
        self.assertFalse(handed['handoff']['world_complete'])

    def test_system_failures_cannot_be_blocked_even_with_two_different_goals(self):
        investigation=self.create();ids=[]
        for x,reason in ((3,'localization_stale'),(5,'revalidated_action_over_budget')):
            task,_=self.submit(investigation['investigation_id'],poses=[{**self.pose,'x':x}])
            ids.append(task['task_id'])
            self.store.update(task['task_id'],status='aborted',reason=reason,stopped=True,
                              result={'map_epoch':'epoch','planning':{'clearance':{'safe':False,'reason':'body_sweep_nonfree'}}})
        with self.assertRaisesRegex(ValueError,'non_route_failure_requires_unresolved'):
            finish(self.store,self.mid,investigation['investigation_id'],'blocked','Not executable',ids)
        result=finish(self.store,self.mid,investigation['investigation_id'],'unresolved','Feedback unavailable',ids)
        self.assertEqual(result['status'],'unresolved')

    def test_plain_path_rejected_string_is_not_obstruction_evidence(self):
        t={'status':'aborted','accepted':True,'stopped':True,'reason':'path_rejected','result':{}}
        self.assertFalse(failure_evidence(t)['supports_blocked'])
        t['result']['planning']={'failure_category':'planner_system_error'}
        self.assertFalse(failure_evidence(t)['supports_blocked'])
        t['result']['planning']={'failure_category':'planner_no_path'}
        self.assertTrue(failure_evidence(t)['supports_blocked'])


if __name__=='__main__':unittest.main()
