"""Safety boundary: long regional paths cannot bypass segment replanning."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from robot_skills.ros_worker import Executor


class RegionalExecutorTests(unittest.TestCase):
    def test_wide_live_path_rejects_body_witness_despite_lidar_clearance(self):
        worker=object.__new__(Executor);worker.candidate_search='wide_4_5'
        worker.grid_snapshot=Mock(return_value={'data':np.zeros((4,4)),'resolution':1.,'origin':[0,0,0]})
        worker.pose=Mock(return_value=[0,0,0])
        witness={'safe':False,'reason':'body_sweep_nonfree','witness_pose':[1,0,0]}
        with patch('robot_skills.ros_worker.Explorer.evaluate_path',return_value={'safe':True,'required_clearance_m':2.15}),\
             patch('observation.body_sweep',return_value=witness) as body:
            result=worker.evaluate_path([[0,0,0],[2,0,0]],{'x':2,'y':0,'yaw':0})
        self.assertFalse(result['safe']);self.assertEqual(result['body_sweep'],witness)
        self.assertEqual(result['required_clearance_m'],2.15);body.assert_called_once()

    def worker(self):
        worker=object.__new__(Executor)
        from aggressive import settings
        worker.config=settings();worker.short_start=False
        worker.ready=Mock();worker.health=Mock();worker.store=Mock()
        worker.store.tasks.return_value=[];worker.store.meta.return_value={}
        worker.pose=Mock(return_value=[0,0,0]);worker.latest={'map_version':'v1'}
        worker.sensor_offset=Mock(return_value=[-.7,0]);worker.freshness=Mock(return_value={'scan':True})
        worker.grid_snapshot=Mock(return_value={'data':np.zeros((2,2)),'resolution':1.,'origin':[0,0,0],
                                               'frame':'map','stamp_sim_s':10.})
        candidate={'id':'far','x':10.,'y':0.,'yaw':0.,'dx':10.,'dy':0.,
                   'region_id':'R_2_0','estimated_gain_m2':2.,'euclidean_distance_m':10.,'frontier_distance_m':1.}
        model=Mock();model.candidates.return_value=([candidate],{'shortlist_truncated':False});model.is_safe.return_value=True
        model.frontier=np.zeros((2,2),dtype=bool);model.resolution=1.
        worker.model=Mock(return_value=model);worker.nav=Mock()
        worker.plan=Mock()
        return worker

    def plan(self,length):
        return {'length':length,'path':[[float(x),0.,0.] for x in range(int(length)+1)],
                'clearance':{'safe':True,'map_version':'v1'}}

    def test_long_goal_publishes_only_replanned_bounded_segment(self):
        worker=self.worker();worker.plan.side_effect=[(self.plan(10),{}),(self.plan(5),{})]
        with patch('observation.Visibility') as visibility:
            visibility.return_value.novel.return_value=1.
            catalog=worker.catalog()
        self.assertEqual(worker.plan.call_count,2)
        candidate=catalog['candidates'][0]
        self.assertTrue(candidate['transit'])
        self.assertEqual(candidate['x'],5.)
        self.assertEqual(candidate['target_region_id'],'R_2_0')
        self.assertEqual(candidate['planned_length_m'],5.)
        self.assertEqual(candidate['target_distance_tier'],'distant')
        self.assertIn(candidate['frontier_id'],catalog['frontier_horizons']['distant'])
        self.assertFalse(catalog['unknown_space']['world_complete'])
        worker.nav.send_goal_async.assert_not_called()

    def test_transit_unknown_analysis_is_recomputed_for_actual_endpoint(self):
        worker=self.worker();worker.plan.side_effect=[(self.plan(10),{}),(self.plan(5),{})]
        with patch('observation.Visibility') as visibility:
            visibility.return_value.novel.return_value=1.
            visibility.return_value.analyze.side_effect=[{'view':'target'},{'view':'relay'}]
            catalog=worker.catalog()
            self.assertEqual(visibility.return_value.analyze.call_args_list[0].args[0],[10.,0.,0.])
            self.assertEqual(visibility.return_value.analyze.call_args_list[1].args[0],[5.,0.,0.])
        candidate=catalog['candidates'][0]
        self.assertEqual(candidate['unknown_analysis'],{'view':'relay'})
        self.assertEqual(candidate['regional_target_unknown_analysis'],{'view':'target'})

    def test_unsafe_or_oversized_replanned_segment_is_never_offered(self):
        for second in ((None,{'reason':'unsafe'}),(self.plan(9),{})):
            worker=self.worker();worker.plan.side_effect=[(self.plan(10),{})]+[second]*6
            with patch('observation.Visibility') as visibility:
                visibility.return_value.novel.return_value=1.
                catalog=worker.catalog()
            self.assertEqual(catalog['candidates'],[])
            self.assertEqual(catalog['rejected'][0]['reason'],'no_safe_transit_pose')
            self.assertTrue(all(a['reason']=='transit_path_rejected' for a in catalog['rejected'][0]['transit_attempts']))
            worker.nav.send_goal_async.assert_not_called()

    def test_unsafe_farthest_pose_tries_an_earlier_replanned_pose(self):
        worker=self.worker()
        worker.model().is_safe.side_effect=[False,True]
        worker.plan.side_effect=[(self.plan(10),{}),(self.plan(4),{})]
        with patch('observation.Visibility') as visibility:
            visibility.return_value.novel.return_value=1.
            catalog=worker.catalog()
        self.assertEqual(catalog['candidates'][0]['x'],4.)
        self.assertEqual(worker.plan.call_count,2)
        worker.nav.send_goal_async.assert_not_called()

    def test_rejected_segment_tries_another_fully_checked_segment(self):
        worker=self.worker()
        worker.plan.side_effect=[(self.plan(10),{}),(None,{'reason':'unsafe'}),(self.plan(4),{})]
        with patch('observation.Visibility') as visibility:
            visibility.return_value.novel.return_value=1.
            catalog=worker.catalog()
        self.assertEqual(catalog['candidates'][0]['x'],4.)
        self.assertEqual(worker.plan.call_count,3)
        worker.nav.send_goal_async.assert_not_called()

    def test_transit_search_timeout_does_not_claim_candidate_unreachable(self):
        worker=self.worker();worker.plan.return_value=(self.plan(10),{})
        with patch('observation.Visibility') as visibility, patch('robot_skills.ros_worker.time.monotonic', side_effect=[0,0,46]):
            visibility.return_value.novel.return_value=1.
            catalog=worker.catalog()
        self.assertEqual(catalog['status'],'candidate_search_incomplete')
        self.assertEqual(catalog['next_candidate_offset'],0)
        self.assertTrue(catalog['more_candidates'])
        self.assertFalse(catalog['search_complete'])
        self.assertEqual(catalog['rejected'],[])
        worker.nav.send_goal_async.assert_not_called()

    def test_unsafe_complete_route_cannot_be_salvaged_by_unchecked_prefix(self):
        worker=self.worker();worker.plan.return_value=(None,{'clearance':{'safe':False}})
        with patch('observation.Visibility'):
            catalog=worker.catalog()
        self.assertEqual(catalog['candidates'],[])
        self.assertEqual(worker.plan.call_count,1)
        worker.nav.send_goal_async.assert_not_called()

    def upgrade_worker(self,mode):
        from frontier import Settings
        worker=self.worker();worker.model().settings=Settings()
        worker.store.meta.side_effect=lambda key,default=None: (
            'mission' if key=='active_mission' else {'exploration_mode':mode} if key=='mission:mission' else default)
        bridge=Mock();bridge.audit=[];bridge.plans=[]
        deny={'retain_for_safety_recheck':False,'reason':'RECENT_LOW_GAIN_OBSERVATION'}
        bridge.enrich.side_effect=lambda c,p:({**c,'exploration_upgrade':{'policy':deny}},False)
        bridge.advise.return_value=deny
        bridge.summary.return_value={'mode':mode,'mission_id':'mission','policy_rejected_count':1}
        worker.upgrade_bridge=Mock(return_value=bridge)
        return worker

    def test_shadow_denial_does_not_filter_baseline_candidate_or_score(self):
        worker=self.upgrade_worker('shadow');worker.plan.side_effect=[(self.plan(10),{}),(self.plan(5),{})]
        with patch('observation.Visibility') as visibility:
            visibility.return_value.novel.return_value=1.
            catalog=worker.catalog()
        self.assertEqual(len(catalog['candidates']),1)
        self.assertEqual(catalog['candidates'][0]['x'],5.)
        self.assertGreater(catalog['candidates'][0]['classical_score'],0)
        self.assertEqual(catalog['status'],'ready')
        worker.nav.send_goal_async.assert_not_called()

    def test_memory_does_not_send_a_transit_towards_a_suppressed_final_goal(self):
        worker=self.upgrade_worker('memory');worker.plan.return_value=(self.plan(10),{})
        with patch('observation.Visibility'):
            catalog=worker.catalog()
        self.assertEqual(catalog['candidates'],[])
        self.assertEqual(catalog['status'],'no_safe_informative_candidate')
        self.assertEqual(worker.plan.call_count,1)
        self.assertEqual(worker.model().candidates.call_args.args[1],[])
        worker.nav.send_goal_async.assert_not_called()

    def test_latest_policy_is_rechecked_in_executor_before_any_dispatch(self):
        worker=self.worker();worker.task={'exploration_mode':'memory','mission_id':'mission'}
        worker.store.meta.return_value={'id':'epoch'}
        bridge=Mock();bridge.advise.return_value={'retain_for_safety_recheck':False,'reason':'RECENT_LOW_GAIN_OBSERVATION'}
        worker.upgrade_bridge=Mock(return_value=bridge)
        with self.assertRaisesRegex(RuntimeError,'exploration_policy_rejected'):
            worker.recheck_exploration_policy({'x':5.,'y':0.,'yaw':0.,'region_epoch':'epoch'},self.plan(5))
        worker.nav.send_goal_async.assert_not_called()


if __name__=='__main__': unittest.main()
