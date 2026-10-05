"""Host facade for a fixed simulation trial; request IDs never replay movement."""
import json
from pathlib import Path
import time
import uuid
import sys

ROOT = Path(__file__).resolve().parents[2]
DIRECTORY = ROOT / 'workspace/log/motion-trials'


def identifier(value):
    parsed = str(uuid.UUID(value))
    if parsed != value:
        raise ValueError('request_id_must_be_canonical_uuid')
    return parsed


def result_for(request_id):
    request_id = identifier(request_id)
    result = DIRECTORY / (request_id + '.result.json')
    if result.exists():
        return json.loads(result.read_text())
    return {'request_id': request_id, 'status': 'running', 'stopped': False}


def _legacy_cancel_motion_trial(request_id):
    request_id = identifier(request_id)
    if not (DIRECTORY / (request_id + '.request')).exists():
        return {'status': 'unknown_request'}
    (DIRECTORY / (request_id + '.cancel')).touch()
    return {**result_for(request_id), 'cancel_requested': True}


def _skills():
    sys.path.insert(0, str(ROOT / 'workspace'))
    from robot_skills.api import RobotSkills
    return RobotSkills()


def cancel_motion_trial(request_id):
    request_id = identifier(request_id)
    if (DIRECTORY / (request_id + '.request')).exists():
        return _legacy_cancel_motion_trial(request_id)
    skills = _skills()
    task = skills.store.by_request(request_id)
    if task is None:
        return {'status': 'unknown_request'}
    return skills.store.cancel(task['task_id'])


def move_forward_trial(request_id):
    """Compatibility entry: historical results replay; new requests use Robot Skills."""
    request_id = identifier(request_id)
    if (DIRECTORY / (request_id + '.request')).exists():
        return {**result_for(request_id), 'replayed_without_movement': True}
    if DIRECTORY.exists():
        if any(not p.with_suffix('.result.json').exists() for p in DIRECTORY.glob('*.request')):
            return {'status': 'rejected', 'reason': 'unresolved_previous_trial'}
        if any(json.loads(p.read_text()).get('status') in ('stop_unconfirmed','indeterminate')
               for p in DIRECTORY.glob('*.result.json')):
            return {'status': 'rejected', 'reason': 'previous_stop_unconfirmed'}
    skills = _skills()
    task = skills.fixed_step(request_id)
    deadline = time.monotonic()+320
    while task['status'] in ('accepted','running','stopping'):
        if time.monotonic() > deadline:
            skills.stop_robot(str(uuid.uuid4()))
            return {'status': 'indeterminate', 'stopped': False, 'task_id': task['task_id']}
        time.sleep(.5)
        task = skills.get_task_status(task['task_id'])
    return {**(task.get('result') or {}), **task}
