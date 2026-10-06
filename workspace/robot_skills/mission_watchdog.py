"""Detached wall/simulation/distance watchdog; does not call any model."""
import fcntl
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_skills.api import RobotSkills
from robot_skills import mission
from robot_skills.store import ACTIVE


def run(mid):
    skills = RobotSkills(mission_id=mid); store = skills.store
    with (store.directory/(mid+'.watchdog.lock')).open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        while True:
            m = store.meta('mission:'+mid)
            if not m or m['status'] == 'finished': return
            state = skills.get_robot_state(); clock = state.get('sources', {}).get('clock', {})
            why = mission.reason(m, store.tasks(), clock.get('sim_time_s'))
            if state.get('status') != 'available' or not clock.get('fresh'): why = why or 'feedback_unavailable'
            if state.get('stop_latched'): why = why or 'operator_stop'
            if why:
                mission.update(store, mid, status='stopping', stop_reason=why)
                task = skills.stop_robot(m['stop_request_id'])
                deadline = time.monotonic()+85
                while time.monotonic()<deadline:
                    tasks = store.tasks(); task = store.get(task['task_id'])
                    active = [t for t in tasks if t['status'] in ACTIVE]
                    if not active:
                        # Another operator stop may own confirmation if ours was rejected.
                        confirmation = task if task['stopped'] else next((t for t in tasks
                            if t['kind']=='stop_robot' and t['stopped'] and t['created_unix_s']>=m['started_unix_s']), None)
                        mission.update(store, mid, status='finished', stopped=confirmation is not None,
                                       stop_task_id=confirmation['task_id'] if confirmation else task['task_id'],
                                       final_progress=mission.progress(m,tasks))
                        return
                    time.sleep(.3)
                # Preserve the reservation/latch if the stop still has no terminal evidence.
                mission.update(store, mid, stopped=False, watchdog_error='stop_confirmation_timeout')
                return
            time.sleep(.25)


if __name__ == '__main__':
    run(sys.argv[1])
