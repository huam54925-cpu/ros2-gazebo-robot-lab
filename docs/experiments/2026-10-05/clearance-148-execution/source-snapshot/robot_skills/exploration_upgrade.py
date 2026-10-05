"""Local-only mode and terminal-event bridge. No numpy, ROS, or model calls.

The task ledger remains authoritative. The auxiliary database can be rebuilt
from terminal task records; pending records block memory-mode admission.
"""
from dataclasses import asdict
import hashlib
import json
from exploration_support.memory import Event, EventStore

MODES = ('baseline', 'shadow', 'memory')
CANDIDATE_SEARCHES = ('baseline', 'wide_4_5')
TERMINAL = {'succeeded','rejected','aborted','canceled','stop_unconfirmed','indeterminate'}


def mode_for(store, mid=None):
    mid = mid or store.meta('active_mission')
    return (store.meta('mission:'+mid, {}) if mid else {}).get('exploration_mode','baseline')


def search_for(store, mid=None):
    mid = mid or store.meta('active_mission')
    return (store.meta('mission:'+mid, {}) if mid else {}).get('candidate_search','baseline')


def path_clearance_for(store, mid=None):
    mid = mid or store.meta('active_mission')
    return (store.meta('mission:'+mid, {}) if mid else {}).get('path_clearance_m',2.15)


def first_direct_required(m, steps=None):
    return (m.get('candidate_search')=='wide_4_5' and m.get('path_clearance_m',2.15)==2.15
            and (m.get('progress',{}).get('steps',0) if steps is None else steps)==0)


def short_start_for(store, task=None):
    mid=task.get('mission_id') if task else store.meta('active_mission')
    m=store.meta('mission:'+mid,{}) if mid else {}
    if not m.get('short_start') or m.get('candidate_search')!='wide_4_5':return False
    return not any(t.get('mission_id')==mid and t['kind']!='stop_robot'
                   and t['task_id']!=(task or {}).get('task_id') for t in store.tasks())


def owned_tasks(tasks, mid):
    return [t for t in tasks if t.get('mission_id') == mid and t.get('accepted')
            and t['kind']=='execute_frontier' and t['status'] in TERMINAL]


def history_revision(tasks, mid):
    data=sorted((t['task_id'],t['status'],t.get('stopped',False)) for t in owned_tasks(tasks,mid))
    return hashlib.sha256(json.dumps(data).encode()).hexdigest()[:20]


def admission_reason(catalog, tasks, mid, mode):
    """Cheap ledger checks under the existing admission transaction, no planning."""
    audit=catalog.get('exploration_upgrade',{})
    if mode=='baseline':
        return 'exploration_mode_mismatch' if audit.get('mode','baseline')!='baseline' else None
    if audit.get('mode')!=mode or audit.get('mission_id')!=mid:
        return 'exploration_mode_mismatch'
    if mode=='memory':
        if any(t.get('exploration_event_pending',True) for t in owned_tasks(tasks,mid)):
            return 'exploration_memory_pending'
        if audit.get('history_revision')!=history_revision(tasks,mid):
            return 'exploration_history_changed'
    return None


def memory_store(store):
    return EventStore(store.directory/'exploration-memory'/'events.sqlite3')


def publish_terminal(store, task_id):
    """Only local completion/supervision calls this; polling/replays never do."""
    task=store.get(task_id)
    if task.get('exploration_mode','baseline')=='baseline' or not task.get('accepted') or task['kind']!='execute_frontier' or task['status'] not in TERMINAL:
        return False
    c=task.get('candidate') or {}; result=task.get('result') or {}
    evidence=result.get('exploration_evidence') or {}
    actual=(result.get('after') or {}).get('pose') if task['status']=='succeeded' else None
    target=actual or [c.get('x',0.),c.get('y',0.),c.get('yaw',0.)]
    gain=result.get('map_gain') or {}
    comparable=gain.get('usable_for_trend') and actual is not None and task['status']=='succeeded' and task['stopped']
    event=Event(event_id='terminal:'+task_id,mission_id=task['mission_id'],
                map_epoch=c['region_epoch'],step=task['exploration_step'],target=tuple(target),
                status=task['status'],stopped=task['stopped'],
                purpose='wait' if task['status']=='succeeded' and actual is None else 'transit' if c.get('transit') else 'observe_goal',
                gain_m2=gain.get('observed_new_known_area_m2') if comparable else None,
                reason=task.get('reason') or '',
                failure_scope=evidence.get('failure_scope','system' if task['status']!='succeeded' else 'none'),
                approach_key=evidence.get('approach_key',''),patch=evidence.get('patch'),
                frontier_cluster_id=c.get('frontier_cluster_id',''),
                frontier_lineage=tuple(c.get('frontier_lineage',())))
    inserted=memory_store(store).append(event)
    # Separate DB commits are deliberate. A crash leaves pending=True and the
    # idempotent reconciler repeats append before acknowledging the task.
    with store.transaction() as db:
        row=db.execute('SELECT body FROM tasks WHERE id=?',(task_id,)).fetchone()
        latest=json.loads(row[0])
        if latest['status']!=task['status']:
            raise ValueError('terminal_changed_during_memory_commit')
        latest['exploration_event_pending']=False
        latest['exploration_event_id']=event.event_id
        store._write(db,latest)
    return inserted


def reconcile(store, mid):
    """Local catalog/supervisor recovery, never a read-context side effect."""
    for task in owned_tasks(store.tasks(),mid):
        if task.get('exploration_event_pending',True):
            publish_terminal(store,task['task_id'])


def model_view(value):
    """Shadow instrumentation is logged, not shown to the decision maker."""
    if isinstance(value,dict):
        return {k:model_view(v) for k,v in value.items() if not k.startswith('exploration_')}
    if isinstance(value,(list,tuple)): return [model_view(v) for v in value]
    return value
