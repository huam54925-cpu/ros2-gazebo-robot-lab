"""Durable idempotency, task arbitration and stop latch (shared host/container)."""
from contextlib import contextmanager
import json
from pathlib import Path
import sqlite3
import time
import uuid
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

DIRECTORY = Path(__file__).resolve().parents[1] / 'log' / 'robot-skills'
ACTIVE = {'accepted', 'running', 'stopping'}
TERMINAL = {'succeeded', 'rejected', 'aborted', 'canceled', 'stop_unconfirmed', 'indeterminate'}


def valid_uuid(value):
    if not isinstance(value, str) or str(uuid.UUID(value)) != value:
        raise ValueError('canonical_uuid_required')
    return value


class Store:
    def __init__(self, directory=DIRECTORY):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.path = self.directory / 'tasks.sqlite3'
        with self.connect() as db:
            db.execute('CREATE TABLE IF NOT EXISTS tasks (id TEXT PRIMARY KEY, request TEXT UNIQUE, fingerprint TEXT, body TEXT)')
            db.execute('CREATE TABLE IF NOT EXISTS meta (key TEXT PRIMARY KEY, value TEXT)')

    @contextmanager
    def connect(self):
        db = sqlite3.connect(self.path, timeout=10)
        try:
            yield db
            db.commit()
        except BaseException:
            db.rollback()
            raise
        finally:
            db.close()

    @contextmanager
    def transaction(self):
        with self.connect() as db:
            db.execute('BEGIN IMMEDIATE')
            yield db

    def _meta(self, db, key, default=None):
        row = db.execute('SELECT value FROM meta WHERE key=?', (key,)).fetchone()
        return json.loads(row[0]) if row else default

    def _set_meta(self, db, key, value):
        db.execute('INSERT OR REPLACE INTO meta VALUES (?,?)', (key, json.dumps(value, allow_nan=False)))

    def meta(self, key, default=None):
        with self.connect() as db:
            return self._meta(db, key, default)

    def set_meta(self, key, value):
        with self.transaction() as db:
            self._set_meta(db, key, value)

    def tasks(self):
        with self.connect() as db:
            return [json.loads(row[0]) for row in db.execute('SELECT body FROM tasks ORDER BY rowid DESC')]

    def get(self, task_id):
        valid_uuid(task_id)
        with self.connect() as db:
            row = db.execute('SELECT body FROM tasks WHERE id=?', (task_id,)).fetchone()
        if not row:
            raise ValueError('unknown_task')
        return json.loads(row[0])

    def by_request(self, request_id):
        valid_uuid(request_id)
        with self.connect() as db:
            row = db.execute('SELECT body FROM tasks WHERE request=?', (request_id,)).fetchone()
        return json.loads(row[0]) if row else None

    def _write(self, db, body):
        db.execute('UPDATE tasks SET body=? WHERE id=?',
                   (json.dumps(body, allow_nan=False), body['task_id']))

    def update(self, task_id, **changes):
        with self.transaction() as db:
            row = db.execute('SELECT body FROM tasks WHERE id=?', (task_id,)).fetchone()
            if not row:
                raise ValueError('unknown_task')
            body = json.loads(row[0])
            if body['status'] in TERMINAL and not (changes.get('status') == 'indeterminate' and not body['stopped']):
                return body
            body.update(changes)
            if 'sensor_fresh' in changes:
                body['sensor_observed_unix_s'] = time.time()
            body['running'] = body['status'] in ('running', 'stopping')
            body['updated_unix_s'] = time.time()
            from robot_skills.investigation import QUERY_KINDS
            if body['status'] in TERMINAL and body['kind'] not in QUERY_KINDS | {'stop_robot'} and body['accepted']:
                if body.get('exploration_mode','baseline')!='baseline':
                    body['exploration_event_pending']=True
                for key in ('catalog', 'observation_catalog'):
                    catalog = self._meta(db, key, {})
                    catalog['expires_unix_s'] = 0
                    self._set_meta(db, key, catalog)
            if body['status'] in TERMINAL:
                from robot_skills.investigation import record_terminal
                record_terminal(self,db,body)
            self._write(db, body)
            return body

    def submit(self, kind, payload, request_id, mission_id=None):
        valid_uuid(request_id)
        if kind in ('fixed_step','perform_observation'):raise ValueError('retired_skill_use_investigation')
        from robot_skills.investigation import TASK_KINDS, QUERY_KINDS, NAVIGATION_KINDS, poses_payload, repetition_reason, failure_code
        if kind not in TASK_KINDS | {'execute_frontier','stop_robot'}:
            raise ValueError('unknown_skill')
        fingerprint = json.dumps([kind, payload], sort_keys=True, allow_nan=False)
        with self.transaction() as db:
            row = db.execute('SELECT fingerprint,body FROM tasks WHERE request=?', (request_id,)).fetchone()
            if row:
                if row[0] != fingerprint:
                    raise ValueError('request_id_conflict')
                previous=json.loads(row[1])
                if previous.get('mission_id')!=mission_id:
                    raise ValueError('request_id_mission_conflict')
                return {**previous, 'replayed_without_execution': True}, False
            tasks = [json.loads(row[0]) for row in db.execute('SELECT body FROM tasks')]
            from .mission import admission
            reason = admission(self, db, tasks, mission_id) if kind != 'stop_robot' else None
            selected = None
            mode=(self._meta(db,'mission:'+mission_id,{}) if mission_id else {}).get('exploration_mode','baseline')
            search=(self._meta(db,'mission:'+mission_id,{}) if mission_id else {}).get('candidate_search','baseline')
            clearance=(self._meta(db,'mission:'+mission_id,{}) if mission_id else {}).get('path_clearance_m',2.15)
            if kind != 'stop_robot':
                if self._meta(db, 'active_operator_assistance'):
                    reason = 'operator_assistance_active'
                elif self._meta(db, 'stop_latched', False):
                    reason = 'stop_latched'
                elif any(t['status'] in ACTIVE for t in tasks):
                    reason = 'another_task_active'
                elif any(t['status'] in ('stop_unconfirmed', 'indeterminate') and not t.get('resolved') for t in tasks):
                    reason = 'previous_stop_unconfirmed'
                mission = self._meta(db,'mission:'+str(mission_id),{})
                if kind=='execute_frontier' and not mission.get('two_stage'):
                    reason=reason or 'two_stage_mission_required'
                if kind in TASK_KINDS:
                    if not mission.get('two_stage'):
                        reason = reason or 'two_stage_mission_required'
                    from robot_skills.recovery import SHORT_MOTION_KINDS, source_reason
                    if kind in SHORT_MOTION_KINDS:
                        epoch=payload.get('map_epoch')
                        inv=self._meta(db,'investigation:'+str(payload.get('investigation_id')), {})
                        if epoch!=self._meta(db,'regional_epoch',{}).get('id'):
                            reason=reason or 'map_epoch_changed'
                        if (mission.get('phase')!='ai_investigation' or inv.get('mission_id')!=mission_id
                                or inv.get('status')!='active' or inv.get('map_epoch')!=epoch
                                or mission.get('active_investigation')!=payload.get('investigation_id')):
                            reason=reason or 'active_investigation_required'
                        if kind=='recover_short_reverse':
                            source=next((t for t in tasks if t['task_id']==payload.get('source_task_id')),None)
                            reason=reason or source_reason(source,tasks,mission_id,epoch,payload.get('investigation_id'))
                            if any(r['source_task_id']==payload.get('source_task_id') for r in self._meta(db,'recovery_attempts:'+str(mission_id),[])):
                                reason=reason or 'recovery_source_already_attempted'
                        allowed={'map_epoch','investigation_id'} | ({'source_task_id'} if kind=='recover_short_reverse' else set())
                        if set(payload)!=allowed:raise ValueError('invalid_short_motion_payload')
                    if kind in NAVIGATION_KINDS | {'plan_navigation'}:
                        checked=poses_payload(payload.get('poses'),payload.get('map_epoch'))
                        if checked['map_epoch']!=self._meta(db,'regional_epoch',{}).get('id'):
                            reason=reason or 'map_epoch_changed'
                        if kind in NAVIGATION_KINDS:
                            inv=self._meta(db,'investigation:'+str(payload.get('investigation_id')),{})
                            if (mission.get('phase')!='ai_investigation' or inv.get('mission_id')!=mission_id
                                    or inv.get('status')!='active' or inv.get('map_epoch')!=checked['map_epoch']):
                                reason=reason or 'active_investigation_required'
                            selected={**checked['poses'][-1], 'region_epoch':checked['map_epoch'],
                                      'map_version':payload.get('map_version')}
                            memory=self._meta(db,'investigation_memory:'+str(mission_id),[])
                            snapshot=self._meta(db,'map_snapshot',{})
                            reason=reason or repetition_reason(memory,checked['poses'],checked['map_epoch'],snapshot.get('robot_pose'))
                if mission.get('two_stage') and kind=='execute_frontier' and mission.get('phase')!='classical':
                    reason=reason or 'frontier_execution_only_in_classical_phase'
                if kind == 'execute_frontier':
                    catalog = self._meta(db, 'catalog', {})
                    selected = next((c for c in catalog.get('candidates', [])
                                     if c['frontier_id'] == payload.get('frontier_id')), None)
                    if selected is None:
                        reason = reason or 'unknown_frontier_id'
                    elif any(t['kind'] == kind and t['payload'] == payload and t['accepted'] for t in tasks):
                        reason = reason or 'frontier_already_attempted'
                    elif time.time() > catalog.get('expires_unix_s', 0):
                        reason = reason or 'frontier_catalog_expired'
                    if selected is not None:
                        if catalog.get('path_clearance_m',2.15)!=clearance:
                            reason = reason or 'path_clearance_mismatch'
                        if catalog.get('candidate_search','baseline')!=search:
                            reason = reason or 'candidate_search_mismatch'
                        from robot_skills.exploration_upgrade import admission_reason
                        reason = reason or admission_reason(catalog,tasks,mission_id,mode)
            else:
                self._set_meta(db, 'stop_latched', True)
                self._set_meta(db, 'stop_generation', self._meta(db, 'stop_generation', 0)+1)
                for task in tasks:
                    if task['kind'] != 'stop_robot' and task['status'] in ACTIVE:
                        task['cancel_requested'] = True
                        self._write(db, task)
                if any(t['kind'] == 'stop_robot' and t['status'] in ACTIVE for t in tasks):
                    reason = 'stop_already_in_progress'
            task_id = str(uuid.uuid4())
            body = {'request_id': request_id, 'task_id': task_id, 'kind': kind, 'mission_id': mission_id, 'payload': payload,
                    'exploration_mode':mode,
                    'candidate_search':search,
                    'path_clearance_m':clearance,
                    'exploration_step':1+sum(t.get('mission_id')==mission_id and t['kind']!='stop_robot' for t in tasks),
                    'accepted': reason is None, 'status': 'rejected' if reason else 'accepted',
                    'running': False, 'stopped': False, 'reason': reason,
                    'failure_code':failure_code(reason) if reason else None,'source':'task_admission',
                    'sensor_fresh': {}, 'sensor_observed_unix_s': None,
                    'map_version': selected.get('map_version') if selected else None,
                    'candidate': selected, 'cancel_requested': False,
                    'created_unix_s': time.time(), 'updated_unix_s': time.time(), 'result': None}
            db.execute('INSERT INTO tasks VALUES (?,?,?,?)', (task_id, request_id, fingerprint,
                                                            json.dumps(body, allow_nan=False)))
            return body, reason is None

    def cancel(self, task_id):
        return self.update(task_id, cancel_requested=True)

    def should_cancel(self, task_id):
        task = self.get(task_id)
        return task['cancel_requested'] or (task['kind'] != 'stop_robot' and self.meta('stop_latched', False))

    def clear_stop_after_verified_resume(self, generation=None):
        with self.transaction() as db:
            if self._meta(db, 'active_mission'):
                raise ValueError('mission_still_active')
            tasks = [json.loads(row[0]) for row in db.execute('SELECT body FROM tasks')]
            if any(t['status'] in ACTIVE or (t['status'] in {'stop_unconfirmed','indeterminate'} and not t.get('resolved')) for t in tasks):
                raise ValueError('unresolved_task')
            if generation is not None and generation != self._meta(db, 'stop_generation', 0):
                raise ValueError('stop_changed_during_resume')
            self._set_meta(db, 'stop_latched', False)

    def record_stop_recovery(self, task_id, evidence):
        """Operator-only recovery keeps the failure status and original audit record."""
        with self.transaction() as db:
            row = db.execute('SELECT body FROM tasks WHERE id=?', (task_id,)).fetchone()
            if not row:
                raise ValueError('unknown_task')
            body = json.loads(row[0])
            if body['status'] not in ('indeterminate', 'stop_unconfirmed'):
                raise ValueError('task_does_not_need_recovery')
            body.update(resolved=True, stopped=True, recovery_evidence=evidence,
                        recovery_unix_s=time.time(), updated_unix_s=time.time())
            if body['status'] in TERMINAL:
                from robot_skills.investigation import record_terminal
                record_terminal(self,db,body)
            self._write(db, body)
            return body
