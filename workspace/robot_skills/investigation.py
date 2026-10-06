"""Persistent investigations and evidence-based repetition control (no ROS)."""
import json
import math
import time
import uuid

QUERY_KINDS = {'view_map', 'plan_navigation', 'search_frontiers'}
NAVIGATION_KINDS = {'navigate_to_pose', 'navigate_through_poses'}
from .recovery import SHORT_MOTION_KINDS
TASK_KINDS = QUERY_KINDS | NAVIGATION_KINDS | SHORT_MOTION_KINDS
DEFAULT_HANDOFF = {'window': 4, 'low_gain_m2': 0.25, 'empty_complete_searches': 2,
                   'repeat_radius_m': 0.5, 'failure_cooldown_s': 120.0}


def finite_pose(value):
    if not isinstance(value, dict) or set(value) != {'x', 'y', 'yaw'}:
        raise ValueError('pose_requires_x_y_yaw')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
           or abs(v) > 1e5 for v in value.values()):
        raise ValueError('invalid_pose')
    return {**value, 'yaw': math.atan2(math.sin(value['yaw']), math.cos(value['yaw']))}


def poses_payload(poses, map_epoch):
    if not isinstance(poses, list) or not 1 <= len(poses) <= 32:
        raise ValueError('one_to_32_waypoints_per_request')
    if not isinstance(map_epoch, str) or not 1 <= len(map_epoch) <= 80:
        raise ValueError('map_epoch_required')
    return {'poses': [finite_pose(p) for p in poses], 'map_epoch': map_epoch, 'frame_id': 'map'}


def failure_code(reason):
    reason = str(reason or '')
    for words, code in [
        (('stop_unconfirmed',), 'STOP_UNCONFIRMED'),
        (('localization', 'transform'), 'LOCALIZATION_UNAVAILABLE'),
        (('stale', 'feedback', 'scan_invalid'), 'SENSOR_STALE'),
        (('contract', 'profile'), 'CONFIG_MISMATCH'),
        (('epoch',), 'MAP_EPOCH_CHANGED'),
        (('guard_stop', 'scan_body_sweep'), 'COLLISION_MONITOR_STOP'),
        (('unknown', 'outside_known'), 'UNKNOWN_SPACE_BLOCKED'),
        (('body_sweep', 'path_invalidated'), 'PATH_INVALIDATED'),
        (('pose_no_longer_safe', 'unsafe_pose'), 'GOAL_IN_COLLISION'),
        (('no_motion_progress', 'short_motion_no_progress', 'Failed to make progress', 'oscillation'), 'NO_PROGRESS'),
        (('planning_deadline', 'action_response_timeout'), 'PLANNING_DEADLINE'),
        (('goal_timeout',), 'EXECUTION_TIMEOUT'),
        (('budget', 'max_'), 'BUDGET_EXHAUSTED'),
        (('stop_requested', 'operator'), 'CANCELED'),
        (('path_rejected', 'no_path'), 'NO_PATH')]:
        if any(w in reason for w in words):
            return code
    return 'EXECUTION_ERROR'


def task_progress(task):
    result = task.get('result') or {}
    gain = result.get('map_gain') or {}
    amount = gain.get('observed_new_known_area_m2') if gain.get('usable_for_trend') else None
    return {'observed_gain_m2': amount,
            'waypoints_reached': result.get('waypoints_reached', 0),
            'goal_reached': task['kind'] not in SHORT_MOTION_KINDS and task.get('status') == 'succeeded' and task.get('stopped') is True,
            'passage_crossed': result.get('passage_crossed') is True,
            'pose': (result.get('after') or {}).get('pose')}


def record_terminal(store, db, task):
    """Called inside the same transaction as the terminal task write."""
    mid = task.get('mission_id')
    if not mid or not task.get('accepted') or task['kind'] in QUERY_KINDS | {'stop_robot'}:
        return
    mission = store._meta(db, 'mission:'+mid, {})
    if not mission.get('two_stage'):
        return
    key = 'investigation_memory:'+mid
    events = store._meta(db, key, [])
    events=[e for e in events if e['task_id']!=task['task_id']]
    candidate = task.get('candidate') or {}
    result = task.get('result') or {}
    progress = task_progress(task)
    goal={k:candidate.get(k) for k in ('x','y','yaw')}
    previous_goals=[e['goal'] for e in events if e['map_epoch']==candidate.get('region_epoch') and e['progress']['goal_reached']]
    progress['new_waypoint_reached']=bool(progress['goal_reached'] and goal.get('x') is not None and not any(
        old.get('x') is not None and math.hypot(goal['x']-old['x'],goal['y']-old['y'])<.5 for old in previous_goals))
    failed = task['status'] != 'succeeded'
    event = {'task_id': task['task_id'], 'investigation_id': task.get('payload', {}).get('investigation_id'),
             'map_epoch': candidate.get('region_epoch') or result.get('map_epoch'), 'goal': {k: candidate.get(k) for k in ('x', 'y', 'yaw')},
             'approach_pose': (result.get('before') or {}).get('pose'),
             'status': task['status'], 'reason': task.get('reason'), 'transit':candidate.get('transit',False),
             'failure_code': failure_code(task.get('reason')) if failed else None,
             'progress': progress, 'recoveries':result.get('recoveries',[]), 'at_unix_s': time.time(),
             'recheck_after_unix_s': time.time()+mission['handoff_policy']['failure_cooldown_s'],
             'retry_condition': 'changed_approach_or_goal_or_cooldown_with_fresh_planning'}
    events.append(event)
    store._set_meta(db, key, events)
    iid = event['investigation_id']
    if iid:
        inv = store._meta(db, 'investigation:'+iid)
        if inv:
            if task['task_id'] not in inv.setdefault('task_ids', []):
                inv['task_ids'].append(task['task_id'])
            inv['last_progress'] = progress
            inv['updated_unix_s'] = time.time()
            store._set_meta(db, 'investigation:'+iid, inv)


def repetition_reason(events, poses, epoch, approach_pose=None, now=None):
    """Suppress failed approaches, never mark corridors as costmap obstacles."""
    now = time.time() if now is None else now
    goal = poses[-1]
    matches = []
    for event in events:
        old = event.get('goal', {})
        if event.get('map_epoch') != epoch or any(old.get(k) is None for k in ('x', 'y', 'yaw')):
            continue
        same_goal = math.hypot(goal['x']-old['x'], goal['y']-old['y']) < .35
        same_heading = abs(math.atan2(math.sin(goal['yaw']-old['yaw']), math.cos(goal['yaw']-old['yaw']))) < .35
        old_start = event.get('approach_pose')
        same_approach = not approach_pose or not old_start or math.dist(approach_pose[:2], old_start[:2]) < .75
        if same_goal and same_heading and same_approach:
            matches.append(event)
    if any(e.get('failure_code') and now < e['recheck_after_unix_s'] for e in matches):
        return 'failed_approach_cooldown'
    scoped=[e for e in events if e.get('map_epoch')==epoch]
    if oscillating(scoped) and any(
            e.get('goal',{}).get('x') is not None and
            math.hypot(goal['x']-e['goal']['x'],goal['y']-e['goal']['y'])<.5 for e in scoped[-4:]):
        return 'oscillation_requires_new_observation_goal'
    return None


def oscillating(events, window=4, radius=.5):
    rows = events[-window:]
    if len(rows) < window:
        return False
    # Uncomparable map feedback is not zero gain.
    if any(e['progress'].get('observed_gain_m2') is None for e in rows):
        return False
    if any(e['progress']['observed_gain_m2'] > .05 or e['progress'].get('passage_crossed')
           or e['progress'].get('new_waypoint_reached') for e in rows):
        return False
    poses = [e['progress'].get('pose') for e in rows]
    if any(p is None for p in poses):
        return False
    # A -> B -> A -> B with no new information. A single useful transit is allowed.
    return all(math.dist(poses[i][:2], poses[i-2][:2]) < radius for i in range(2, len(poses)))


def create(store, mid, subject, hypothesis, task_type, request_id, passage=None):
    from .store import valid_uuid
    valid_uuid(request_id)
    if task_type not in ('observe_structure', 'verify_passage'):
        raise ValueError('invalid_investigation_type')
    subject = finite_pose(subject)
    if not isinstance(hypothesis, str) or not 1 <= len(hypothesis) <= 1500:
        raise ValueError('hypothesis_required')
    if task_type == 'verify_passage':
        if not isinstance(passage, dict) or set(passage) != {'a', 'b', 'destination_side'}:
            raise ValueError('passage_requires_entry_segment_and_destination_side')
        if passage['destination_side'] not in (-1, 1):
            raise ValueError('invalid_destination_side')
        for p in (passage['a'], passage['b']):
            if not isinstance(p, list) or len(p) != 2 or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in p):
                raise ValueError('invalid_entry_segment')
        if math.dist(passage['a'], passage['b']) < .1:
            raise ValueError('invalid_entry_segment')
    elif passage is not None:
        raise ValueError('passage_only_for_verify_passage')
    fp = json.dumps([mid, subject, hypothesis, task_type, passage], sort_keys=True)
    with store.transaction() as db:
        previous = store._meta(db, 'investigation_request:'+request_id)
        if previous:
            if previous['fingerprint'] != fp:
                raise ValueError('request_id_conflict')
            return store._meta(db, 'investigation:'+previous['id'])
        m = store._meta(db, 'mission:'+str(mid), {})
        if not m.get('two_stage') or m.get('phase') != 'ai_investigation' or m.get('status') != 'running':
            raise ValueError('ai_investigation_phase_required')
        if store._meta(db, 'stop_latched', False):
            raise ValueError('stop_latched')
        active = m.get('active_investigation')
        if active and store._meta(db, 'investigation:'+active, {}).get('status') == 'active':
            raise ValueError('finish_current_investigation_first')
        epoch = store._meta(db, 'regional_epoch', {}).get('id')
        if not epoch:
            raise ValueError('map_snapshot_required')
        iid = str(uuid.uuid4())
        inv = {'investigation_id': iid, 'mission_id': mid, 'subject': subject, 'hypothesis': hypothesis,
               'type': task_type, 'passage': passage, 'status': 'active', 'map_epoch': epoch,
               'task_ids': [], 'created_unix_s': time.time(), 'updated_unix_s': time.time()}
        store._set_meta(db, 'investigation:'+iid, inv)
        store._set_meta(db, 'investigation_request:'+request_id, {'fingerprint': fp, 'id': iid})
        m['active_investigation'] = iid
        m.setdefault('investigation_ids', []).append(iid)
        store._set_meta(db, 'mission:'+mid, m)
        return inv


def finish(store, mid, iid, outcome, assessment, evidence_ids):
    if outcome not in ('observations_collected', 'passage_verified', 'blocked', 'unresolved'):
        raise ValueError('invalid_investigation_outcome')
    if not isinstance(evidence_ids,list) or any(not isinstance(v,str) for v in evidence_ids):
        raise ValueError('evidence_task_ids_must_be_a_list')
    if not isinstance(assessment, str) or not 1 <= len(assessment) <= 2000:
        raise ValueError('assessment_required')
    with store.transaction() as db:
        inv = store._meta(db, 'investigation:'+iid)
        if not inv or inv['mission_id'] != mid:
            raise ValueError('unknown_investigation')
        if inv['status'] != 'active':
            if (inv['status'], inv['model_assessment'], inv['evidence_task_ids']) != (outcome, assessment, evidence_ids):
                raise ValueError('investigation_already_finished')
            return inv
        tasks = [json.loads(r[0]) for r in db.execute('SELECT body FROM tasks')]
        from .store import ACTIVE
        if any(t['status'] in ACTIVE and t['mission_id'] == mid for t in tasks):
            raise ValueError('task_still_active')
        owned = [t for t in tasks if t['mission_id'] == mid and t.get('payload', {}).get('investigation_id') == iid]
        evidence = [t for t in owned if t['task_id'] in evidence_ids]
        if len(evidence) != len(set(evidence_ids)):
            raise ValueError('evidence_must_belong_to_investigation')
        if any(t.get('dispatched') and not t.get('stopped') for t in evidence):
            raise ValueError('stop_unconfirmed')
        if outcome == 'observations_collected' and not any(
                (t.get('result') or {}).get('map_gain', {}).get('sensor_feedback_fresh') and
                (t.get('result') or {}).get('map_gain', {}).get('map_message_advanced') and t.get('stopped') for t in evidence):
            raise ValueError('fresh_observation_evidence_required')
        if outcome == 'passage_verified' and (inv['type'] != 'verify_passage' or not any(
                (t.get('result') or {}).get('passage_crossed') and t.get('stopped') for t in evidence)):
            raise ValueError('whole_body_passage_evidence_required')
        if outcome == 'blocked':
            failed = [t for t in evidence if t['status'] in ('rejected', 'aborted') and t.get('accepted')
                      and t['kind']!='recover_short_reverse' and t.get('candidate')
                      and all((t['candidate'].get(k) is not None) for k in ('x','y','yaw'))]
            goals = {(round((t.get('candidate') or {}).get('x',0),1),
                      round((t.get('candidate') or {}).get('y',0),1),
                      round((t.get('candidate') or {}).get('yaw',0),1)) for t in failed}
            if len(goals) < 2:
                raise ValueError('blocked_requires_two_distinct_attempts')
        inv.update(status=outcome, model_assessment=assessment, evidence_task_ids=evidence_ids,
                   updated_unix_s=time.time(), structure_confirmed=False,
                   evidence_scope='online_observations_and_estimated_pose; assessment_is_model_interpretation')
        store._set_meta(db, 'investigation:'+iid, inv)
        return inv


def handoff_reason(mission, events, catalog):
    policy = mission['handoff_policy']
    rows = events[-policy['window']:]
    if len(rows) == policy['window'] and all(e['progress'].get('observed_gain_m2') is not None for e in rows):
        if (sum(e['progress']['observed_gain_m2'] for e in rows) <= policy['low_gain_m2']
                and not any(e.get('transit') and e['progress'].get('new_waypoint_reached') for e in rows)):
            return 'sustained_low_gain'
    if oscillating(events, policy['window'], policy['repeat_radius_m']):
        return 'repeated_without_progress'
    if mission.get('empty_complete_searches', 0) >= policy['empty_complete_searches']:
        return 'bounded_search_exhausted'
    return None


def transition(store, mid, catalog, snapshot):
    """Only called by the deterministic mission owner after a completed search."""
    with store.transaction() as db:
        m = store._meta(db, 'mission:'+mid)
        if not m or m.get('phase') != 'classical' or m['status'] != 'running':
            raise ValueError('classical_phase_required')
        from .store import ACTIVE
        if any(json.loads(r[0])['status'] in ACTIVE for r in db.execute('SELECT body FROM tasks')):
            raise ValueError('handoff_requires_no_active_task')
        if store._meta(db, 'stop_latched', False):
            raise ValueError('stop_latched')
        completed = catalog.get('search_complete') is True and not catalog.get('more_candidates')
        healthy_search = catalog.get('status') in ('ready', 'no_safe_reachable_frontiers', 'no_safe_informative_candidate')
        if not healthy_search:
            return m
        search_id = catalog.get('catalog_id') or str(catalog.get('created_unix_s'))
        if search_id != m.get('last_handoff_search_id'):
            m['last_handoff_search_id'] = search_id
            m['empty_complete_searches'] = m.get('empty_complete_searches', 0)+1 if completed and not catalog.get('candidates') else 0
        events = store._meta(db, 'investigation_memory:'+mid, [])
        why = handoff_reason(m, events, catalog)
        if why:
            if not snapshot or not snapshot.get('stopped') or not snapshot.get('map_epoch'):
                raise ValueError('handoff_snapshot_required')
            m.update(phase='ai_investigation', handoff={'reason': why, 'at_unix_s': time.time(),
                     'snapshot': snapshot, 'task_ids': [e['task_id'] for e in events],
                     'world_complete': False})
        store._set_meta(db, 'mission:'+mid, m)
        return m
