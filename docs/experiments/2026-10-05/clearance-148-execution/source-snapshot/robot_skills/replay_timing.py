"""Reproduce time estimates from archived candidate paths, without ROS/motion."""
import argparse
import json
import math
from pathlib import Path
import sys

WORKSPACE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WORKSPACE/'navigation_base/exploration'))
from observation import action_time_estimate


def replay(report_path, store):
    report=json.loads(Path(report_path).read_text())
    rows=[]
    for row in report['rounds']:
        task=row['execution_result']; result=task['result']; candidate=task['candidate']
        catalog=json.loads((store/'exploration-upgrade'/(row['candidate_set']['catalog_id']+'.json')).read_text())
        path=next(p['checked_path'] for p in catalog['plans_and_advice']
                  if p['candidate'].get('frontier_id')==candidate['frontier_id'])
        estimate=action_time_estimate(candidate['planned_length_m'],path,catalog['map_metadata']['resolution'])
        before,after=result['before'],result['after']
        trace=json.loads((store/(task['task_id']+'.detail.trace.json')).read_text())
        totals=dict(translating_including_curves=0.,turning_in_place=0.,stationary=0.)
        for a,b in zip(trace,trace[1:]):
            dt=b['simulation_seconds']-a['simulation_seconds']
            if not 0<=dt<=2:continue
            v,w=a['velocity']
            key='translating_including_curves' if abs(v)>=.02 else 'turning_in_place' if abs(w)>=.02 else 'stationary'
            totals[key]+=dt
        rows.append({'step':row['index'],'task_id':task['task_id'],'old_estimate_sim_s':candidate['estimated_sim_seconds'],
            'recomputed':estimate,'observed_task_window_sim_s':after['simulation_seconds']-before['simulation_seconds'],
            'observed_task_window_wall_s':after['wall_elapsed']-before['wall_elapsed'],
            'round_wall_s':row['elapsed_wall_s'],'model_wall_s':row['decision']['latency_wall_s'],
            'feedback_wait_wall_s':result['map_gain']['feedback_wait_wall_s'],
            'first_navigation_sample_minus_before_sim_s':trace[0]['simulation_seconds']-before['simulation_seconds'],
            'sampled_navigation_sim_s':totals,'sampled_navigation_scope':'left_sample_velocity; sparse trace, not exact phase boundaries',
            'exact_planning_sim_s':None,'exact_stop_sim_s':None,'exact_model_sim_s':None,
            'measured_gain_m2':result['map_gain'].get('observed_new_known_area_m2') if result['map_gain'].get('usable_for_trend') else None})
    final=report['last_decision_context']['session_budget']
    final_id=report['last_candidate_set']['catalog_id']
    catalog=json.loads((store/'exploration-upgrade'/(final_id+'.json')).read_text())
    estimates=[]
    for c in report['last_candidate_set']['candidates']:
        path=next(p['checked_path'] for p in catalog['plans_and_advice'] if p['candidate'].get('frontier_id')==c['frontier_id'])
        estimate=action_time_estimate(c['planned_length_m'],path,catalog['map_metadata']['resolution'])
        estimates.append({'id':c['frontier_id'],'old_sim_s':c['estimated_sim_seconds'],**estimate})
    return {'scope':'offline; original logs unchanged; no motion','steps':rows,
            'stop_budget':{'remaining_sim_s':final['remaining_sim_s'],'candidates':estimates,
                'fits_revised_before_model':sum(e['budget_sim_s']<=final['remaining_sim_s'] for e in estimates)},
            'limitations':['Two windows cannot calibrate a general speed model.',
                'Historical logs lack exact plan/stop/model simulation phase boundaries; null is intentional.',
                'A counterfactual fit does not prove dispatch feasibility after new planning/model waits.']}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report',type=Path,required=True)
    parser.add_argument('--store',type=Path,default=WORKSPACE/'log/robot-skills')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();data=replay(args.report,args.store)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'steps':[{k:r[k] for k in ('step','old_estimate_sim_s','observed_task_window_sim_s')} |
            {'new_expected_sim_s':r['recomputed']['expected_sim_s'],'new_budget_sim_s':r['recomputed']['budget_sim_s']} for r in data['steps']],
            'fits_revised_before_model':data['stop_budget']['fits_revised_before_model']},indent=2))
