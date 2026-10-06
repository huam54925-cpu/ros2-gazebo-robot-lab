"""Local-only bounded sessions. Model tools cannot set budgets or release locks."""
import math
import time
import uuid


def limits(max_steps=3, max_distance=6., max_sim_time=120., max_wall_time=180., max_failures=2, profile='trial'):
    if profile == 'mission':
        if max_steps is not None:
            raise ValueError('mission_uses_time_distance_failure_budgets_not_steps')
        values = dict(max_steps=None, max_distance=max_distance, max_sim_time=max_sim_time,
                      max_wall_time=max_wall_time, max_failures=max_failures, profile=profile)
        for key in ('max_distance','max_sim_time','max_wall_time','max_failures'):
            if isinstance(values[key],bool) or not math.isfinite(values[key]) or values[key] <= 0:
                raise ValueError('invalid_'+key)
        if int(max_failures)!=max_failures:
            raise ValueError('invalid_max_failures')
        return values
    if profile not in ('trial','extended','hour'): raise ValueError('invalid_budget_profile')
    values = dict(max_steps=max_steps, max_distance=max_distance, max_sim_time=max_sim_time,
                  max_wall_time=max_wall_time, max_failures=max_failures)
    for name, value in values.items():
        if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
            raise ValueError('invalid_'+name)
    caps = {'trial':(5,20,180,900), 'extended':(20,80,900,3600),
            'hour':(60,240,3600,3600)}[profile]
    if int(max_steps) != max_steps or max_steps > caps[0] or int(max_failures) != max_failures or max_failures > 3:
        raise ValueError('step_or_failure_limit_out_of_range')
    if max_distance > caps[1] or max_sim_time > caps[2] or max_wall_time > caps[3]:
        raise ValueError('session_budget_out_of_range')
    return {**values,'profile':profile}


def extend_operator_budget(store, mid, budget, sim_time):
    """Explicit local operator action only; never a model/MCP capability.

    Preserve start clocks, accumulated tasks/distance and stop state. Expired or
    stopping missions cannot be revived by giving them another budget.
    """
    import json
    budget=limits(**budget)
    if budget['profile']=='mission' or store.meta('mission:'+mid,{}).get('two_stage'):
        raise ValueError('two_stage_budget_is_fixed_for_the_run')
    with store.transaction() as db:
        m=store._meta(db,'mission:'+mid)
        if not m or store._meta(db,'active_mission')!=mid or store._meta(db,'stop_latched',False):
            raise ValueError('mission_not_running_or_stop_latched')
        tasks=[json.loads(row[0]) for row in db.execute('SELECT body FROM tasks')]
        if reason(m,tasks,sim_time): raise ValueError('cannot_extend_ending_mission')
        if m.get('exploration_mode','baseline')!='baseline' and budget['max_steps']>3 and budget['profile'] not in ('extended','hour'):
            raise ValueError('upgrade_requires_at_most_three_tasks')
        if m.get('path_clearance_m',2.15)==1.48 and budget['max_steps']>3:
            raise ValueError('reduced_clearance_requires_at_most_three_wide_frontier_tasks')
        old=m['limits']
        if any(budget[k]<old[k] for k in old if k!='profile'):
            raise ValueError('extension_cannot_reduce_limits')
        if budget['max_failures']!=old['max_failures']:
            raise ValueError('extension_preserves_failure_limit')
        m.setdefault('initial_limits',old)
        m.setdefault('budget_changes',[]).append({'at_unix_s':time.time(), 'source':'explicit_local_operator',
                                                 'previous_limits':old,'new_limits':budget})
        m['limits']=budget
        m['deadline_monotonic_s']=m['started_monotonic_s']+budget['max_wall_time']
        store._set_meta(db,'mission:'+mid,m)
        return m


def progress(mission, tasks):
    from .investigation import QUERY_KINDS
    owned = [t for t in tasks if t.get('mission_id') == mission['mission_id']
             and t['kind'] not in QUERY_KINDS | {'stop_robot'}]
    return {'steps': len(owned),
            'distance_m': sum(max(t.get('distance_odom_m', 0), (t.get('result') or {}).get('distance_odom_m', 0)) for t in owned),
            'failures': sum(t['status'] in ('rejected','aborted','canceled','indeterminate','stop_unconfirmed') for t in owned),
            'active': any(t['status'] in ('accepted','running','stopping') for t in owned)}


def reason(mission, tasks, sim_time=None, now=None, check_steps=True):
    if mission['status'] != 'running':
        return mission.get('stop_reason') or 'mission_not_running'
    now = time.monotonic() if now is None else now
    budget = mission['limits']; p = progress(mission, tasks)
    if now >= mission['deadline_monotonic_s']: return 'max_wall_time'
    if now-mission['heartbeat_monotonic_s'] > 15: return 'client_heartbeat_lost'
    if sim_time is not None:
        if sim_time < mission['start_sim_s']-.1: return 'simulation_clock_reset'
        if sim_time-mission['start_sim_s'] >= budget['max_sim_time']: return 'max_sim_time'
    if p['distance_m'] >= budget['max_distance']: return 'max_distance'
    if p['failures'] >= budget['max_failures']: return 'max_failures'
    if check_steps and budget['max_steps'] is not None and not p['active'] and p['steps'] >= budget['max_steps']: return 'max_steps'
    return None


def action_budget(mission, tasks, sim_time, estimated_sim_s, path_length_m):
    """Last executor check after planning; no dispatch if the action no longer fits."""
    if not all(math.isfinite(v) for v in (sim_time,estimated_sim_s,path_length_m)) or min(estimated_sim_s,path_length_m)<0:
        raise ValueError('invalid_action_budget_input')
    used=progress(mission,tasks)
    left=mission['limits']['max_sim_time']-(sim_time-mission['start_sim_s'])
    distance=mission['limits']['max_distance']-used['distance_m']
    reasons=[]
    if estimated_sim_s>left:reasons.append('time_budget')
    if path_length_m>distance:reasons.append('distance_budget')
    return {'fits':not reasons,'reasons':reasons,'remaining_sim_s':left,
            'remaining_distance_m':distance,'estimated_sim_s':estimated_sim_s,'planned_length_m':path_length_m}


def start(store, budget, state, observations=False, exploration_mode='baseline', candidate_search='baseline', short_start=False, path_clearance_m=2.15, two_stage=False):
    budget = limits(**budget)
    from navigation_base.safety_profile import PROFILE,FOOTPRINT_MODE
    if two_stage:
        if budget['profile']!='mission' or not FOOTPRINT_MODE or path_clearance_m!=2.15:
            raise ValueError('two_stage_requires_mission_budget_and_footprint_profile')
        if observations or exploration_mode!='baseline' or candidate_search!='baseline' or short_start:
            raise ValueError('two_stage_uses_persistent_investigation_policy')
    elif budget['profile']=='mission':
        raise ValueError('mission_budget_requires_two_stage')
    if not two_stage and FOOTPRINT_MODE and (budget['max_steps']>3 or observations):
        raise ValueError('footprint_trial_requires_at_most_three_navigation_tasks')
    if path_clearance_m not in (1.48,2.15): raise ValueError('invalid_path_clearance')
    if path_clearance_m==1.48 and (budget['max_steps']>3 or candidate_search!='wide_4_5' or observations):
        raise ValueError('reduced_clearance_requires_at_most_three_wide_frontier_tasks')
    from .exploration_upgrade import CANDIDATE_SEARCHES
    if candidate_search not in CANDIDATE_SEARCHES: raise ValueError('invalid_candidate_search')
    if candidate_search != 'baseline' and exploration_mode != 'memory':
        raise ValueError('wide_search_requires_memory_trial')
    if not isinstance(short_start,bool) or (short_start and candidate_search!='wide_4_5'):
        raise ValueError('short_start_requires_wide_search')
    if exploration_mode not in ('baseline','shadow','memory'): raise ValueError('invalid_exploration_mode')
    if exploration_mode!='baseline' and (observations or (budget['max_steps']>3 and budget['profile'] not in ('extended','hour'))):
        raise ValueError('upgrade_requires_frontier_only_at_most_three_tasks')
    if state.get('status') != 'available': raise ValueError('robot_unavailable')
    clock = state.get('sources', {}).get('clock', {})
    odom = state.get('sources', {}).get('odometry', {})
    if not clock.get('fresh') or not odom.get('fresh'): raise ValueError('feedback_unavailable')
    if any(not math.isfinite(v) or abs(v) >= .02 for v in odom.get('velocity', {'missing': 1}).values()): raise ValueError('robot_not_stationary')
    with store.transaction() as db:
        if store._meta(db, 'active_operator_assistance'): raise ValueError('operator_assistance_active')
        if store._meta(db, 'active_mission'): raise ValueError('another_mission_active')
        if store._meta(db, 'stop_latched', False): raise ValueError('stop_latched')
        import json
        tasks = [json.loads(r[0]) for r in db.execute('SELECT body FROM tasks')]
        if any(t['status'] in ('accepted','running','stopping') or
               (t['status'] in ('indeterminate','stop_unconfirmed') and not t.get('resolved')) for t in tasks):
            raise ValueError('unresolved_task')
        now = time.monotonic(); mid = str(uuid.uuid4())
        body = {'mission_id': mid, 'status': 'running', 'stop_reason': None, 'limits': budget,
                'safety_profile':PROFILE,'exploration_mode':exploration_mode,
                'two_stage':two_stage,'phase':'classical' if two_stage else 'legacy',
                'candidate_search':candidate_search,
                'short_start':short_start,
                'path_clearance_m':path_clearance_m,
                'observations_enabled': observations, 'start_sim_s': clock['sim_time_s'],
                'started_monotonic_s': now, 'deadline_monotonic_s': now+budget['max_wall_time'],
                'heartbeat_monotonic_s': now, 'stop_request_id': str(uuid.uuid4()),
                'stopped': False, 'started_unix_s': time.time()}
        if two_stage:
            from .investigation import DEFAULT_HANDOFF
            from navigation_base.robot_contract import CONTRACT_HASH
            body.update(handoff_policy=dict(DEFAULT_HANDOFF),contract_hash=CONTRACT_HASH,
                        empty_complete_searches=0,investigation_ids=[])
        store._set_meta(db, 'mission:'+mid, body); store._set_meta(db, 'active_mission', mid)
    return body


def update(store, mid, **changes):
    with store.transaction() as db:
        body = store._meta(db, 'mission:'+mid)
        if not body: raise ValueError('unknown_mission')
        if body['status'] == 'finished': return body
        # A late heartbeat cannot resurrect an ending session.
        if body['status'] == 'stopping' and changes.get('status') == 'running': changes.pop('status')
        if body.get('stop_reason') and changes.get('stop_reason'): changes.pop('stop_reason')
        body.update(changes)
        if body.get('two_stage') and body['status'] in ('stopping','finished'):
            body['phase']='stop_confirmation' if body['status']=='stopping' else 'finished'
            if body['status']=='finished' and body.get('active_investigation'):
                key='investigation:'+body['active_investigation']
                investigation=store._meta(db,key,{})
                if investigation.get('status')=='active':
                    investigation.update(status='unresolved',termination_reason=body.get('stop_reason'),
                                         updated_unix_s=time.time(),structure_confirmed=False,
                                         stopped=body.get('stopped',False))
                    store._set_meta(db,key,investigation)
        store._set_meta(db, 'mission:'+mid, body)
        if body['status'] == 'finished' and store._meta(db, 'active_mission') == mid:
            store._set_meta(db, 'active_mission', None)
        return body


def admission(store, db, tasks, mid):
    active = store._meta(db, 'active_mission')
    if active and active != mid: return 'another_mission_active'
    if mid:
        body = store._meta(db, 'mission:'+mid)
        if not body or active != mid: return 'mission_not_active'
        return reason(body, tasks)
    return None
