import asyncio
import unittest
from pathlib import Path
import subprocess
import sys
from unittest.mock import patch
from run_loop import decision_loop,validate_choice,classic_choice,first_action_validation,can_reselect_after_body_stop

class LoopTests(unittest.IsolatedAsyncioTestCase):
    def test_first_wide_action_requires_verified_gain_and_stop(self):
        import copy
        good={'status':'succeeded','stopped':True,'candidate':{'transit':False},
              'result':{'map_gain':{'usable_for_trend':True,'observed_new_known_area_m2':1.}}}
        self.assertIsNone(first_action_validation(good))
        for value in (None,0.,-1.,float('nan')):
            task=copy.deepcopy(good);task['result']['map_gain']['observed_new_known_area_m2']=value
            self.assertIsNotNone(first_action_validation(task))
        for field,value in [('status','aborted'),('stopped',False),('candidate',{'transit':True})]:
            self.assertIsNotNone(first_action_validation({**good,field:value}))
        task=copy.deepcopy(good);task['result']['map_gain']['usable_for_trend']=False
        self.assertEqual(first_action_validation(task),'first_action_gain_unverified')

    def test_extended_first_goal_keeps_motion_checks_and_null_gain(self):
        task={'status':'succeeded','stopped':True,'candidate':{'transit':False},
              'result':{'map_gain':{'usable_for_trend':False,'observed_new_known_area_m2':None}}}
        self.assertIsNone(first_action_validation(task,require_gain=False))
        self.assertIsNone(task['result']['map_gain']['observed_new_known_area_m2'])
        for field,value in [('status','aborted'),('stopped',False),('candidate',{'transit':True})]:
            self.assertIsNotNone(first_action_validation({**task,field:value},require_gain=False))

    async def test_wide_trial_filters_first_transit_and_stops_on_unverified_gain(self):
        m={'status':'running','candidate_search':'wide_4_5','limits':{'max_distance':10},'progress':{'steps':0}}
        report={'rounds':[]};submitted=[];reasons=[]
        async def call(name,args=None):
            if name=='get_safe_frontiers':return {'candidate_search':'wide_4_5','candidates':[
                {'frontier_id':'relay','planned_length_m':1,'transit':True},
                {'frontier_id':'direct','planned_length_m':2,'transit':False}]}
            if name=='execute_frontier':
                submitted.append(args['frontier_id'])
                return {'task_id':'task','status':'succeeded','stopped':True,
                        'candidate':{'transit':False},'result':{'map_gain':{'usable_for_trend':False}}}
            return {}
        async def choose(context,available,previous):
            self.assertEqual(list(available),['direct'])
            return {'action':'MOVE','option_id':'direct','rationale_summary':'first direct test'},{}
        def end(why):reasons.append(why);m.update(status='stopping')
        await decision_loop(call,choose,lambda:m,end,lambda:None,report)
        self.assertEqual(submitted,['direct'])
        self.assertEqual(reasons,['first_action_gain_unverified'])
        self.assertFalse(report['rounds'][0]['first_action_validation']['passed'])

    async def test_hour_body_abort_reselects_only_after_confirmed_stop(self):
        for profile in ('trial','extended','hour'):
            m={'limits':{'profile':profile}}
            task={'status':'aborted','stopped':True,'reason':'body_sweep_nonfree'}
            self.assertEqual(can_reselect_after_body_stop(m,task),profile=='hour')
            self.assertFalse(can_reselect_after_body_stop(m,{**task,'stopped':False}))
            self.assertFalse(can_reselect_after_body_stop(m,{**task,'reason':'guard_stop'}))
        for locked in (False,True):
            report={'rounds':[]};submitted=[];choices=[];reasons=[]
            m={'status':'running','candidate_search':'wide_4_5','limits':{'max_distance':10,'profile':'hour'},'progress':{'steps':0}}
            async def call(name,args=None):
                if name=='get_safe_frontiers':return {'candidate_search':'wide_4_5','candidates':[{'frontier_id':'new' if submitted else 'first','planned_length_m':1,'transit':False}]}
                if name=='execute_frontier':
                    submitted.append(args['frontier_id']);m['progress']={'steps':1}
                    return {'task_id':'a','status':'aborted','stopped':True,'reason':'body_sweep_nonfree','candidate':{'transit':False},'result':{}}
                if name=='get_robot_state':return {'stop_latched':locked and bool(submitted)}
                return {}
            async def choose(context,available,previous):
                choices.append(list(available))
                return ({'action':'STOP','option_id':'STOP','rationale_summary':'test complete'} if submitted else
                        {'action':'MOVE','option_id':'first','rationale_summary':'test first'}),{}
            def end(why):reasons.append(why);m.update(status='stopping')
            await decision_loop(call,choose,lambda:m,end,lambda:None,report)
            self.assertEqual(submitted,['first'])
            self.assertEqual(choices,[['first']] if locked else [['first'],['new']])
            self.assertEqual(reasons,['operator_stop'] if locked else ['model_stop'])

    async def test_rejected_candidate_batch_searches_next_batch_before_stopping(self):
        report={'rounds':[]};m={'status':'running','limits':{'max_distance':6}};batches=0;choices=[]
        async def call(name,args=None):
            nonlocal batches
            if name=='get_safe_frontiers':
                batches+=1
                return {'status':'candidate_search_incomplete','candidates':[],'more_candidates':True} if batches==1 else {'candidates':[{'frontier_id':'ALT','planned_length_m':2}]}
            return {}
        async def choose(context,available,previous):
            choices.extend(available)
            return {'action':'STOP','option_id':'STOP','rationale_summary':'test finished'},{}
        await decision_loop(call,choose,lambda:m,lambda why:m.update(status='stopping'),lambda:None,report)
        self.assertEqual(batches,2);self.assertEqual(choices,['ALT'])
        self.assertEqual(len(report['additional_searches']),1)

    async def test_readonly_model_connection_retry_refreshes_context_without_moving(self):
        import openai,httpx2
        report={'rounds':[]};m={'status':'running','limits':{'max_distance':6}};decisions=0;calls=[]
        async def call(name,args=None):
            calls.append(name)
            return {'candidates':[{'frontier_id':'F','planned_length_m':1}]} if name=='get_safe_frontiers' else {}
        async def choose(*args):
            nonlocal decisions
            decisions+=1
            if decisions==1: raise openai.APIConnectionError(request=httpx2.Request('POST','https://api.openai.com/v1/responses'))
            return {'action':'STOP','option_id':'STOP','rationale_summary':'test stop'},{}
        await decision_loop(call,choose,lambda:m,lambda why:m.update(status='stopping'),lambda:None,report)
        self.assertEqual(decisions,2);self.assertEqual(calls.count('get_safe_frontiers'),2)
        self.assertNotIn('execute_frontier',calls)
        self.assertEqual(report['rounds'][0]['decision_error']['status'],'api_connection_error')

    async def test_empty_candidate_search_is_bounded(self):
        batches=0;reasons=[]
        async def call(name,args=None):
            nonlocal batches
            if name=='get_safe_frontiers': batches+=1;return {'status':'candidate_search_incomplete','more_candidates':True,'candidates':[]}
            return {}
        async def choose(*args):self.fail('No safe ID exists')
        await decision_loop(call,choose,lambda:{'status':'running','limits':{'max_distance':6}},reasons.append,lambda:None,{'rounds':[]})
        self.assertEqual(batches,3);self.assertEqual(reasons,['candidate_generation_unavailable'])
    async def test_three_sequential_goals_refresh_context_and_unique_requests(self):
        calls=[]; ids=[]; report={'rounds':[]}; m={'status':'running','limits':{'max_distance':6},'progress':{'distance_m':0}}
        completed=0; context_count=0
        async def call(name,args=None):
            nonlocal completed,context_count
            calls.append(name)
            if name=='get_decision_context':
                context_count+=1; return {'version':context_count}
            if name=='get_safe_frontiers': return {'candidates':[{'frontier_id':str(completed),'planned_length_m':1}]}
            if name=='execute_frontier':
                ids.append(args['request_id']); return {'task_id':str(completed),'status':'running'}
            if name=='get_task_status':
                completed+=1
                if completed==3: m['status']='stopping'
                return {'task_id':str(completed-1),'status':'succeeded','stopped':True,'result':{'distance_odom_m':1}}
            if name=='get_robot_state': return {'stop_latched':False}
            self.fail(name)
        async def choose(context,available,previous):
            key=next(iter(available)); return {'action':'MOVE','option_id':key,'rationale_summary':'short safe route'},{}
        async def no_wait(*a): pass
        with patch('run_loop.asyncio.sleep',no_wait):
            await decision_loop(call,choose,lambda:m,lambda why:m.update(status='stopping'),lambda:None,report)
        self.assertEqual(completed,3); self.assertEqual(len(set(ids)),3)
        self.assertEqual(context_count,9)
        self.assertEqual([r['model_choice']['option_id'] for r in report['rounds']],['0','1','2'])
        self.assertEqual(calls.count('get_task_status'),3)
    async def test_budget_expires_while_model_waits_no_dispatch(self):
        m={'status':'running','limits':{'max_distance':6}}; sent=[]
        async def call(name,args=None):
            sent.append(name)
            return {'candidates':[{'frontier_id':'F','planned_length_m':1}]} if name=='get_safe_frontiers' else {}
        async def choose(*args):
            m['status']='stopping'; return {'action':'MOVE','option_id':'F','rationale_summary':'x'},{}
        await decision_loop(call,choose,lambda:m,lambda why:None,lambda:None,{'rounds':[]})
        self.assertNotIn('execute_frontier',sent)

    async def test_remaining_action_budget_rechecked_after_model_before_expiry(self):
        m={'status':'running','start_sim_s':0,'limits':{'max_distance':6,'max_sim_time':100}}
        sent=[];reasons=[]
        async def call(name,args=None):
            sent.append(name)
            if name=='get_safe_frontiers':return {'candidates':[
                {'frontier_id':'F','planned_length_m':1,'estimated_sim_seconds':35},
                {'frontier_id':'SHORT','planned_length_m':1,'estimated_sim_seconds':20}]}
            if name=='get_decision_context':return {'robot':{'sources':{'clock':{'sim_time_s':60}}}}
            if name=='get_robot_state':return {'sources':{'clock':{'sim_time_s':70}}}
            return {}
        async def choose(*args):return {'action':'MOVE','option_id':'F','rationale_summary':'x'},{}
        report={'rounds':[]}
        await decision_loop(call,choose,lambda:m,reasons.append,lambda:None,report,max_reselections=0)
        self.assertNotIn('execute_frontier',sent)
        self.assertEqual(reasons,['reselection_limit_reached'])
        self.assertEqual(report['rounds'][0]['selection_attempts'][0]['fitting_ids'],['SHORT'])
        self.assertEqual(report['rounds'][0]['last_choice_rejection'],'selected_action_over_budget')

    def test_model_patch_compaction_preserves_null_gain_and_digest(self):
        from run_loop import compact_model_data
        original={'gain_m2':None,'patch':{'points':[[1,2]],'labels':[0],'digest':'d'}}
        compact=compact_model_data(original)
        self.assertIsNone(compact['gain_m2'])
        self.assertEqual(compact['patch']['digest'],'d')
        self.assertNotIn('points',compact['patch'])
        self.assertIn('points',original['patch'])
    async def test_empty_set_terminates_without_model(self):
        reasons=[]
        async def call(*args): return {}
        async def choose(*args): self.fail('model should not be called')
        await decision_loop(call,choose,lambda:{'status':'running','limits':{'max_distance':6}},reasons.append,lambda:None,{'rounds':[]})
        self.assertEqual(reasons,['no_safe_frontiers'])
    def test_wrong_action_and_extra_fields_rejected(self):
        available={'F':('MOVE',{})}
        for value in ({'action':'OBSERVE','option_id':'F','rationale_summary':'x'},
                      {'action':'MOVE','option_id':'F','rationale_summary':'x','speed':1}):
            with self.assertRaises(ValueError): validate_choice(value,available)

    def test_joint_rule_can_choose_move_or_observe_on_relative_gain(self):
        available={'F':('MOVE',{'joint_score':.5}), 'O':('OBSERVE',{'score':.3})}
        self.assertEqual(classic_choice(available,True),'F')
        available['F'][1]['joint_score']=.1
        self.assertEqual(classic_choice(available,True),'O')

    async def test_model_stop_reason_is_set_before_stop_tool(self):
        m={'status':'running','limits':{'max_distance':6}}; reasons=[]
        async def call(name,args=None):
            if name=='get_safe_frontiers': return {'candidates':[{'frontier_id':'F','planned_length_m':1}]}
            if name=='stop_robot':
                self.assertEqual(reasons,['model_stop'])
                return {'task_id':'stop'}
            return {}
        async def choose(*args): return {'action':'STOP','option_id':'STOP','rationale_summary':'budget'},{}
        await decision_loop(call,choose,lambda:m,reasons.append,lambda:None,{'rounds':[]})

    def test_sigterm_runs_async_cleanup_instead_of_leaving_running_report(self):
        code='''
import asyncio
from run_loop import install_termination_handler
async def main():
    remove=install_termination_handler()
    try:
        print('READY',flush=True)
        await asyncio.sleep(30)
    except asyncio.CancelledError:
        print('INTERRUPTED',flush=True)
    finally:
        await asyncio.sleep(.01)
        print('STOP_AND_CHECKPOINT',flush=True)
        remove()
asyncio.run(main())
'''
        process=subprocess.Popen([sys.executable,'-c',code],cwd=Path(__file__).resolve().parent,
                                 stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
        try:
            self.assertEqual(process.stdout.readline().strip(),'READY')
            process.terminate()
            out,err=process.communicate(timeout=10)
            self.assertEqual(process.returncode,0,err)
            self.assertIn('INTERRUPTED',out)
            self.assertIn('STOP_AND_CHECKPOINT',out)
        finally:
            if process.poll() is None: process.kill();process.wait()

if __name__=='__main__': unittest.main()
