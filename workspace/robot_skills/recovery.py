"""Bounded recovery policy and fixed-heading sweeps. No ROS or command output."""
import json
import math
import time

from navigation_base.robot_contract import CONTRACT

POLICY = CONTRACT['short_motion']
SHORT_MOTION_KINDS = {'probe_forward', 'recover_short_reverse'}


def angle(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def eligible(reason):
    # Only failures with affirmative motion/geometry evidence are recoverable.
    # Timeouts, cancellation, stale feedback and generic Nav2 errors are not.
    return str(reason) in {'no_motion_progress', 'short_motion_no_progress',
                          'guard_stop', 'scan_body_sweep', 'body_sweep_nonfree',
                          'path_invalidated_before_dispatch', 'Failed to make progress'}


def straight_retreat(trace, current, sim_time, stationary_bridge=None):
    """Length of a recent straight forward odometry suffix that can be retraced.

    All points are in the same odometry frame, never mixed with SLAM poses.
    Do not infer a route across sampling gaps, turns, resets or previous reverse.
    """
    if not trace or sim_time < trace[-1]['sim_s'] or sim_time-trace[-1]['sim_s'] > POLICY['trace_gap_s']:
        raise ValueError('recovery_trace_stale')
    if math.dist(current[:2], trace[-1]['pose'][:2]) > POLICY['endpoint_tolerance_m']:
        raise ValueError('recovery_trace_disconnected')
    c,s=math.cos(current[2]),math.sin(current[2]);previous_time=sim_time;distance=0.
    previous_pose=current;frame=trace[-1]['frame']
    for row in reversed(trace):
        p=row['pose'];stamp=row['sim_s']
        if (not all(math.isfinite(v) for v in [*p, stamp]) or stamp>previous_time
                or row['frame']!=frame or sim_time-stamp>POLICY['trace_max_age_s']):
            break
        if previous_time-stamp>POLICY['trace_gap_s']:
            # Only the exact inter-worker stationary boundary, confirmed stopped
            # at both ends, may have missing samples. Never bridge movement.
            if (stationary_bridge!=(stamp,previous_time)
                    or math.dist(p[:2],previous_pose[:2])>POLICY['endpoint_tolerance_m']
                    or abs(angle(p[2],previous_pose[2]))>POLICY['heading_tolerance_rad']):break
        dx,dy=p[0]-current[0],p[1]-current[1]
        behind=-(dx*c+dy*s);lateral=abs(-dx*s+dy*c)
        if (lateral>POLICY['lateral_tolerance_m'] or abs(angle(p[2],current[2]))>POLICY['heading_tolerance_rad']
                or behind < distance-.005):
            break
        distance=max(distance,behind);previous_time=stamp;previous_pose=p
        if distance>=POLICY['reverse_max_m']+POLICY['endpoint_tolerance_m']:break
    distance=min(POLICY['reverse_max_m'],distance-POLICY['endpoint_tolerance_m'])
    if distance < POLICY['minimum_step_m']:
        raise ValueError('recovery_no_recent_straight_route')
    return distance


def straight_sweep(grid, pose, signed_distance):
    """Fixed heading, including reverse: never invent a 180-degree body turn."""
    import numpy as np
    from navigation_base.exploration.observation import body_check
    body=np.asarray(CONTRACT['footprint']);step=CONTRACT['motion_sample_step_m']
    count=max(1,math.ceil(abs(signed_distance)/step))
    for i in range(count+1):
        d=signed_distance*i/count
        p=[pose[0]+d*math.cos(pose[2]),pose[1]+d*math.sin(pose[2]),pose[2]]
        evidence=body_check(grid['data'],grid['resolution'],grid['origin'],p,body,CONTRACT['body_padding_m'])
        if not evidence['safe']:
            return {'safe':False,'reason':evidence['reason'],'witness_pose':p,'cell_evidence':evidence}
    return {'safe':True,'reason':'clear','checked_samples':count+1}


def source_reason(source, tasks, mid, epoch, investigation_id):
    """Explicit recovery may only use the most recent owned failed motion."""
    from .investigation import QUERY_KINDS, NAVIGATION_KINDS
    if not source or source.get('mission_id')!=mid or source['kind'] not in NAVIGATION_KINDS | {'probe_forward'}:
        return 'recovery_source_not_owned_motion'
    result=source.get('result') or {}
    if source.get('payload',{}).get('investigation_id')!=investigation_id:
        return 'recovery_source_investigation_mismatch'
    if source['status'] not in ('aborted','rejected') or not source.get('dispatched') or not source.get('stopped'):
        return 'recovery_source_not_stopped_failure'
    if not eligible(source.get('reason')) or result.get('map_epoch')!=epoch:
        return 'recovery_source_not_eligible'
    motion=[t for t in tasks if t.get('mission_id')==mid and t.get('accepted')
            and t['kind'] not in QUERY_KINDS | {'stop_robot','recover_short_reverse'}]
    if motion and max(motion,key=lambda t:t['created_unix_s'])['task_id']!=source['task_id']:
        return 'recovery_source_superseded'
    return None


def reserve(store, task, pose, epoch, source_id):
    """Consume an attempt before dispatch, atomically with stop/mission checks.

    Even a denied sweep consumes the source attempt: repeated calls cannot probe
    the same failure forever or reset limits by starting another investigation.
    """
    from .mission import admission
    mid=task['mission_id']
    with store.transaction() as db:
        tasks=[json.loads(r[0]) for r in db.execute('SELECT body FROM tasks')]
        why=admission(store,db,tasks,mid)
        owned=next((t for t in tasks if t['task_id']==task['task_id']),None)
        if (why or store._meta(db,'stop_latched',False) or not owned
                or store._meta(db,'active_operator_assistance') or owned.get('cancel_requested') or owned['status']!='running'):
            raise RuntimeError(why or 'stop_requested')
        key='recovery_attempts:'+mid;rows=store._meta(db,key,[])
        if any(r['source_task_id']==source_id for r in rows):
            raise RuntimeError('recovery_source_already_attempted')
        if len(rows)>=POLICY['max_recoveries']:
            raise RuntimeError('max_recoveries')
        nearby=[r for r in rows if r['map_epoch']==epoch and math.dist(pose[:2],r['pose'][:2])<POLICY['location_radius_m']]
        if len(nearby)>=POLICY['max_recoveries_per_location']:
            raise RuntimeError('max_recoveries_at_location')
        row={'attempt':len(rows)+1,'task_id':task['task_id'],'source_task_id':source_id,
             'map_epoch':epoch,'pose':pose,'status':'reserved','created_unix_s':time.time()}
        rows.append(row);store._set_meta(db,key,rows)
        return row


def update_attempt(store, mid, attempt_id, **changes):
    with store.transaction() as db:
        key='recovery_attempts:'+mid;rows=store._meta(db,key,[])
        row=next(r for r in rows if r['attempt']==attempt_id)
        row.update(changes,updated_unix_s=time.time());store._set_meta(db,key,rows)


def probe_reason(tasks, mid, pose, epoch, now=None):
    now=time.time() if now is None else now
    for t in tasks:
        if t.get('mission_id')!=mid or t['kind']!='probe_forward' or not t.get('accepted'):continue
        r=t.get('result') or {};start=(r.get('before') or {}).get('pose')
        if (start and r.get('map_epoch')==epoch and math.dist(start[:2],pose[:2])<.35
                and abs(angle(start[2],pose[2]))<.35
                and now-t['updated_unix_s']<POLICY['probe_cooldown_s']):
            return 'probe_approach_cooldown'
    return None


def reverse_steps(history_limit):
    """Longest permitted suffix first, then smaller 5 cm steps, never past history."""
    if not math.isfinite(history_limit) or history_limit<POLICY['minimum_step_m']:return []
    limit=min(history_limit,POLICY['reverse_max_m'])
    values=[limit]
    n=math.floor((limit-1e-9)/POLICY['reverse_step_m'])
    values.extend(i*POLICY['reverse_step_m'] for i in range(n,0,-1)
                  if i*POLICY['reverse_step_m']>=POLICY['minimum_step_m']-1e-9)
    return values
