import unittest
from navigation_base.exploration.regions import diverse_candidates, region_id, transit_candidate, transit_candidates, transit_exclusions
from robot_skills.regional import context, rank


class RegionalTests(unittest.TestCase):
    def candidate(self, x, score=1):
        return {'x':x, 'y':1., 'yaw':0., 'euclidean_distance_m':x,
                'preplan_score':score, 'region_id':region_id([x,1]),
                'frontier_id':str(x), 'region_epoch':'epoch',
                'regional_potential_proxy_m2':.1, 'regional_score':score}

    def task(self, index, usable=True, status='succeeded', transit=False, epoch='epoch'):
        return {'task_id':str(index), 'created_unix_s':index, 'status':status, 'mission_id':'m',
                'candidate':{**self.candidate(1), 'region_epoch':epoch,
                    'target_region_id':'R_2_0' if transit else 'R_0_0', 'transit':transit},
                'result':{'before':{'pose':[1,1,0]}, 'after':{'pose':[2,1,0]},
                    'map_gain':{'usable_for_trend':usable, 'observed_new_known_area_m2':.1}}}

    def catalog(self):
        return {'region_epoch':'epoch', 'robot_pose':[1,1,0], 'search_complete':True,
                'candidates':[self.candidate(2,10), self.candidate(9,1)]}

    def test_high_scoring_near_duplicates_cannot_hide_distant_region(self):
        candidates=[self.candidate(x,100) for x in (1,1.8,2.6,3.4)] + [self.candidate(5),self.candidate(10)]
        chosen=diverse_candidates(candidates,3)
        self.assertEqual({c['region_id'] for c in chosen},{'R_0_0','R_1_0','R_2_0'})

    def test_sector_ids_are_world_fixed_and_handle_negative_coordinates(self):
        self.assertEqual(region_id([-.1,4.1]),'R_-1_1')

    def test_known_corridor_relocation_keeps_far_region_and_bounded_segment(self):
        c=self.candidate(14)
        relay=transit_candidate(c,[[x,1,0] for x in range(1,15)],[0,1,0])
        self.assertEqual(relay['x'],5)
        self.assertEqual(relay['target_region_id'],'R_3_0')
        self.assertEqual(relay['region_id'],'R_1_0')
        self.assertEqual(relay['euclidean_distance_m'],5.)
        self.assertTrue(relay['transit'])
        self.assertIsNone(transit_candidate(c,[[x,1,0] for x in range(1,6)],[0,1,0],[[x,1] for x in range(2,6)]))

    def test_invalid_or_failed_measurements_never_establish_saturation(self):
        for tasks in ([self.task(i,usable=False) for i in range(3)],
                      [self.task(i,status='aborted') for i in range(3)],
                      [self.task(i,epoch='old') for i in range(3)]):
            result=context(self.catalog(),tasks,'m')
            self.assertFalse(result['local_saturated'])
            self.assertEqual(result['local_gain_evidence'],'INSUFFICIENT')

    def test_transit_options_are_spaced_bounded_and_preserve_destination(self):
        c=self.candidate(14)
        options=transit_candidates(c,[[x/10,1,0] for x in range(1,141)],[0,1,0])
        self.assertLessEqual(len(options),6)
        self.assertGreater(len(options),1)
        self.assertTrue(all(2<=r['x']<=5.5 and r['target_region_id']=='R_3_0' for r in options))
        self.assertTrue(all(a['x']-b['x']>=.5 for a,b in zip(options,options[1:])))

    def test_successful_corridor_reuse_keeps_failures_and_repeat_limit(self):
        goal={**self.candidate(9),'region_epoch':'epoch'}
        first=self.task(1,transit=True);first['accepted']=True
        self.assertEqual(transit_exclusions([first],goal),[])
        second=self.task(2,transit=True);second['accepted']=True
        self.assertEqual(transit_exclusions([first,second],goal),[[1,1.],[1,1.]])
        second['candidate']['target_region_id']='R_5_0'
        self.assertEqual(transit_exclusions([first,second],goal),[])
        second['status']='aborted'
        self.assertEqual(transit_exclusions([first,second],goal),[[1,1.]])

    def test_three_valid_low_windows_and_low_local_potential_expand(self):
        result=context(self.catalog(),[self.task(i) for i in range(3)],'m')
        self.assertTrue(result['local_saturated'])
        self.assertEqual(result['recommended_mode'],'EXPAND_REGION')

    def test_repeated_visits_diversify_without_claiming_saturation(self):
        result=context(self.catalog(),[self.task(i,usable=False) for i in range(2)],'m')
        self.assertFalse(result['local_saturated'])
        self.assertEqual(result['recommendation_reason'],'diversify_after_repeated_local_arrivals')
        available={str(x):('MOVE',self.candidate(x,100 if x==2 else 1)) for x in (2,9)}
        self.assertEqual(rank(available,result),'9')

    def test_low_immediate_gain_transit_commitment_and_failure_release(self):
        catalog=self.catalog()
        result=context(catalog,[self.task(1,transit=True)],'m')
        self.assertEqual(result['committed_region_id'],'R_2_0')
        self.assertIsNone(context(catalog,[self.task(1,transit=True),self.task(2,status='aborted',transit=True)],'m')['committed_region_id'])
        self.assertIsNone(context(catalog,[self.task(i,transit=True) for i in range(3)],'m')['committed_region_id'])

    def test_absent_candidates_never_claim_world_complete(self):
        result=context({'robot_pose':[1,1,0]},[])
        self.assertFalse(result['search_complete'])
        self.assertFalse(result['coverage_complete'])

    def test_truncated_search_or_along_path_potential_prevents_saturation(self):
        tasks=[self.task(i) for i in range(3)]
        catalog=self.catalog();catalog['search_complete']=False
        self.assertFalse(context(catalog,tasks,'m')['local_saturated'])
        catalog=self.catalog();catalog['candidates'][0]['novel_unknown_area_proxy_m2']=4.
        self.assertFalse(context(catalog,tasks,'m')['local_saturated'])

    def test_rotation_does_not_count_as_arriving_in_a_new_region(self):
        task=self.task(1);task['candidate']['type']='rotate'
        self.assertEqual(context(self.catalog(),[task],'m')['arrivals_by_region'],{})


if __name__=='__main__': unittest.main()
