import asyncio
from copy import deepcopy
import time
import unittest
from run_loop import actions,select_revalidated,decision_loop
from robot_skills.mission import action_budget


class ReselectionTests(unittest.IsolatedAsyncioTestCase):
    async def test_initial_all_candidates_over_budget_never_calls_model(self):
        m={'status':'running','start_sim_s':0.,'limits':{'max_distance':10,'max_sim_time':50}}
        reasons=[]
        async def call(name,args=None):
            if name=='get_safe_frontiers':return {'candidates':[{'frontier_id':'F','planned_length_m':1,'estimated_sim_seconds':10}]}
            return {'robot':{'sources':{'clock':{'sim_time_s':45}}}}
        async def choose(*args):self.fail('No model request for empty feasible set')
        await decision_loop(call,choose,lambda:m,reasons.append,lambda:None,{'rounds':[]})
        self.assertEqual(reasons,['all_candidates_over_budget'])

    async def run_case(self,estimates,delays,limit=2,replace_catalog=False,stop=False,expire=False):
        current=[0.];m={'status':'running','start_sim_s':0.,'limits':{'max_distance':20.,'max_sim_time':50.}}
        catalog={'catalog_id':'first','map_version':'v1','expires_unix_s':time.time()+120,
                 'candidates':[{'frontier_id':str(i),'planned_length_m':1.,'estimated_sim_seconds':v} for i,v in enumerate(estimates)]}
        context={'robot':{'sources':{'clock':{'sim_time_s':0.}}},'session_budget':{'remaining_sim_s':50.,'remaining_distance_m':20.,'used':{}},'frontiers':catalog}
        row={'candidate_set':deepcopy(catalog),'observation_options':{'options':[]},'timing':{}}
        report={'policy':'injected','rounds':[row]};calls=[];choices=[];ended=[]
        async def call(name,args=None):
            calls.append(name)
            if name=='get_decision_context':return {'frontiers':deepcopy(catalog),'active_tasks':[]}
            if name=='get_robot_state':return {'status':'available','stop_latched':stop,'sources':{'clock':{'sim_time_s':current[0],'fresh':True},'map':{'map_version':'v1'}}}
            if name=='get_safe_frontiers':
                catalog.update(expires_unix_s=time.time()+120,catalog_id='refreshed')
                catalog['candidates']=[{**catalog['candidates'][-1],'frontier_id':'new'}]
                return deepcopy(catalog)
            self.fail('Unexpected/motion tool: '+name)
        async def choose(ctx,available,history):
            selected=next(iter(available));choices.append(selected)
            current[0]+=delays[min(len(choices)-1,len(delays)-1)]
            if len(choices)==1 and replace_catalog:
                catalog['candidates']=[{**catalog['candidates'][-1],'frontier_id':'new'}]
            if len(choices)==1 and expire:catalog['expires_unix_s']=time.time()-1
            return {'action':'MOVE','option_id':selected,'rationale_summary':'fixture'}, {'selector':'model_fixture','latency_wall_s':1.}
        value=await select_revalidated(call,choose,lambda:m,ended.append,lambda:None,report,row,context,actions(catalog['candidates'],[]),max_reselections=limit)
        return value,row,ended,choices,calls

    async def test_long_choice_replaced_with_fitting_short_choice(self):
        value,row,ended,choices,calls=await self.run_case([48.,30.],[5.,5.])
        self.assertEqual(choices,['0','1']);self.assertEqual(value[0]['option_id'],'1')
        self.assertEqual(row['timing']['dispatch_remaining_sim_s'],40.)
        self.assertEqual(row['reselection_count'],1);self.assertFalse(ended)

    async def test_all_choices_over_budget_stops_without_reselection(self):
        value,row,ended,choices,_=await self.run_case([48.,47.],[5.])
        self.assertIsNone(value);self.assertEqual(ended,['all_candidates_over_budget']);self.assertEqual(len(choices),1)

    async def test_remaining_action_can_fit_but_not_another_model_call(self):
        value,row,ended,choices,_=await self.run_case([48.,44.],[5.])
        self.assertEqual(ended,['insufficient_time_for_reselection']);self.assertEqual(len(choices),1)

    async def test_limit_stops_even_when_another_shorter_choice_remains(self):
        value,row,ended,choices,_=await self.run_case([48.,35.,10.],[5.,11.],limit=1)
        self.assertEqual(ended,['reselection_limit_reached']);self.assertEqual(len(choices),2)
        self.assertEqual(row['selection_attempts'][-1]['fitting_ids'],['2'])

    async def test_default_two_reselections_never_make_a_fourth_request(self):
        value,row,ended,choices,_=await self.run_case([48.,35.,20.,5.],[5.,11.,15.])
        self.assertIsNone(value);self.assertEqual(ended,['reselection_limit_reached'])
        self.assertEqual(len(choices),3);self.assertEqual(row['reselection_count'],2)
        self.assertEqual(row['selection_attempts'][-1]['fitting_ids'],['3'])

    async def test_deleted_id_never_dispatched_and_new_id_requires_new_choice(self):
        value,row,ended,choices,_=await self.run_case([20.,10.],[1.],replace_catalog=True)
        self.assertEqual(choices,['0','new']);self.assertEqual(value[0]['option_id'],'new')
        self.assertEqual(row['selection_attempts'][0]['validation'],'selected_candidate_no_longer_valid')

    async def test_expired_catalog_is_rebuilt_before_reselection(self):
        value,row,ended,choices,calls=await self.run_case([20.,10.],[1.],expire=True)
        self.assertEqual(calls.count('get_safe_frontiers'),1);self.assertEqual(choices,['0','new'])
        self.assertEqual(value[1]['frontier_id'],'new')

    async def test_stop_latch_outranks_reselection(self):
        value,row,ended,choices,_=await self.run_case([48.,20.],[5.],stop=True)
        self.assertIsNone(value);self.assertEqual(ended,['operator_stop']);self.assertEqual(len(choices),1)

    def test_executor_rechecks_after_planning_time_and_distance_changes(self):
        m={'mission_id':'m','start_sim_s':0,'limits':{'max_distance':10,'max_sim_time':50}}
        self.assertTrue(action_budget(m,[],10,35,2)['fits'])
        self.assertEqual(action_budget(m,[],20,35,2)['reasons'],['time_budget'])
        task={'mission_id':'m','kind':'execute_frontier','status':'succeeded','distance_odom_m':9}
        self.assertEqual(action_budget(m,[task],10,35,2)['reasons'],['distance_budget'])

if __name__=='__main__':unittest.main()
