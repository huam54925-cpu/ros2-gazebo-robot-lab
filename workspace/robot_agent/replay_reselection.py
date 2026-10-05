"""Counterfactual scheduler replay. No ROS, MCP, model request or motion call."""
import argparse
import asyncio
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import time
from unittest.mock import patch
from run_loop import actions,select_revalidated


async def replay(source):
    data=json.loads(source.read_text());record=data['rounds'][1]
    context=deepcopy(record['decision_context']);catalog=deepcopy(record['candidate_set'])
    budget=context['session_budget'];m=deepcopy(data['mission'])
    m.update(status='running',stop_reason=None,progress=budget['used'],deadline_monotonic_s=time.monotonic()+600)
    sim=m['start_sim_s']+m['limits']['max_sim_time']-budget['remaining_sim_s']
    cost=record['timing']['model_and_refresh_sim_s']
    robot=deepcopy(context['robot']);robot['stop_latched']=False
    robot['sources']['clock'].update(sim_time_s=sim,fresh=True)
    context.update(robot=deepcopy(robot),frontiers=catalog,mission=m,active_tasks=[])
    row={'index':1,'candidate_set':catalog,'observation_options':{'options':[]},'timing':{}}
    report={'policy':'recorded_choice_then_replay_fixture','rounds':[row]};ended=[];count=0
    available=actions([c for c in catalog['candidates'] if c['estimated_sim_seconds']<=budget['remaining_sim_s']],[])
    async def call(name,args=None):
        if name=='get_decision_context':return deepcopy({**context,'robot':robot})
        if name=='get_robot_state':return deepcopy(robot)
        raise AssertionError('Replay forbids tool operation: '+name)
    async def choose(ctx,options,previous):
        nonlocal count,sim
        choice=(deepcopy(record['model_choice']) if count==0 else
                {'action':'MOVE','option_id':min(options,key=lambda k:options[k][1]['estimated_sim_seconds']),
                 'rationale_summary':'Offline deterministic fixture: choose shortest fitting action; not a new model response.'})
        sim+=cost;robot['sources']['clock']['sim_time_s']=sim
        selector='recorded_model_choice' if count==0 else 'offline_test_fixture'
        count+=1
        return choice,{'selector':selector,'latency_wall_s':record['decision']['latency_wall_s'],'usage':None}
    with patch('run_loop.time.time',return_value=catalog['created_unix_s']+12):
        selected=await select_revalidated(call,choose,lambda:m,ended.append,lambda:None,report,row,context,available,max_reselections=2)
    result={'source':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'scope':'counterfactual scheduling only; archived geometry and simulated repeated decision delay; no live authorization',
            'assumed_each_decision_and_refresh_sim_s':cost,'model_calls_sent':0,'motion_commands_sent':0,
            'initial_remaining_sim_s':budget['remaining_sim_s'],'stop_reasons':ended,
            'would_select':selected[0]['option_id'] if selected else None,
            'selected_budget_sim_s':selected[1]['estimated_sim_seconds'] if selected else None,
            'remaining_after_reselection_sim_s':row['timing'].get('dispatch_remaining_sim_s'),
            'reselection_count':row.get('reselection_count'),'selection_attempts':row['selection_attempts']}
    expected=next(c['frontier_id'] for c in catalog['candidates'] if c['estimated_sim_seconds']<35)
    assert result['would_select']==expected and result['reselection_count']==1 and not ended
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=asyncio.run(replay(args.source))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='selection_attempts'},ensure_ascii=False,indent=2))
