"""Orchestrator regression: late budget refusal must hand off without resampling."""
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
from investigation_loop import run


class InvestigationLoopTests(unittest.IsolatedAsyncioTestCase):
    async def test_dispatch_reserve_rejection_skips_next_search(self):
        state={'status':'running','phase':'classical'}
        skills=NS(mission_id='m',store=NS(tasks=lambda: []),
                  get_robot_state=lambda: {'sources':{'clock':{'sim_time_s':10.}}})
        calls=[];transitions=[];ends=[]
        async def call(name,args):
            calls.append(name)
            if name=='start_candidate_search':
                return {'status':'succeeded','result':{'catalog':{'candidates':[
                    {'frontier_id':'f','planned_length_m':1.,'estimated_sim_seconds':1.}]}}}
            if name=='view_map':return {'status':'succeeded','result':{'snapshot':{'map_version':'fresh'}}}
            if name=='execute_frontier':return {'status':'rejected','accepted':True,
                'kind':'execute_frontier','reason':'classical_budget_reserved','stopped':True}
            self.fail('unexpected call: '+name)
        def transition(store,mid,catalog,snapshot,reason):
            transitions.append(reason)
            if reason:
                self.assertEqual(snapshot['map_version'],'fresh')
                state.update(phase='ai_investigation',status='finished')
            return state
        with patch('robot_skills.mission.classical_boundary',return_value=None), \
             patch('robot_skills.mission.classical_remaining',return_value={'distance_m':10.,'sim_s':10.}), \
             patch('investigation_loop.transition',side_effect=transition):
            await run(call,[],skills,lambda:state,ends.append,lambda:None,{'rounds':[]},None,None)
        self.assertEqual(calls,['start_candidate_search','view_map','execute_frontier','view_map'])
        self.assertEqual(transitions,[None,'classical_budget_reserved'])
        self.assertEqual(ends,[])


if __name__=='__main__':unittest.main()
