"""Detached local task supervision survives an MCP client disconnect."""
import fcntl
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_skills.api import robot_command
from robot_skills.store import Store, ACTIVE


def main():
    store = Store(); task_id = sys.argv[1]; task = store.get(task_id)
    with (store.directory / (task_id + '.supervisor.lock')).open('w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return
        if task['status'] not in ACTIVE:
            return
        from robot_skills.investigation import QUERY_KINDS
        query=task['kind'] in QUERY_KINDS
        process = None
        fallback = False
        try:
            with (store.directory / (task_id + '.ros.log')).open('w') as log:
                process = subprocess.Popen(robot_command('task', task_id), stdout=log, stderr=log)
                profile=(store.meta('mission:'+task['mission_id'],{}) if task.get('mission_id') else {}).get('safety_profile')
                mission=store.meta('mission:'+str(task.get('mission_id')),{})
                timeout=(65 if task['kind']=='stop_robot' else 260 if query else
                         mission.get('limits',{}).get('max_wall_time',680)+60
                         if mission.get('two_stage') else 680 if profile=='footprint_075' else 290)
                process.wait(timeout=timeout)
            final = store.get(task_id)
            fallback = process.returncode != 0 or final['status'] in ACTIVE or final['status'] == 'stop_unconfirmed'
        except (OSError, subprocess.TimeoutExpired):
            store.cancel(task_id)
            fallback = True
        if fallback and query:
            # Query workers own no velocity source. Never stop Nav2 for an RPC/planning failure.
            if process and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    # Keep the reservation: another task must not overlap an unknown worker.
                    store.update(task_id,status='indeterminate',reason='query_worker_exit_unconfirmed',stopped=False)
                    return
            # Exiting docker exec does not prove the remote worker exited. Wait
            # for its durable terminal acknowledgment before releasing ownership.
            deadline=time.monotonic()+10
            while store.get(task_id)['status'] in ACTIVE and time.monotonic()<deadline:
                time.sleep(.2)
            if store.get(task_id)['status'] in ACTIVE:
                store.update(task_id,status='indeterminate',reason='query_worker_exit_unconfirmed',stopped=False)
            return
        if fallback:
            # Fail closed: remove Nav2 command sources, leaving the independent guard alive.
            store.set_meta('stop_latched', True)
            shutdown_confirmed = False
            try:
                shutdown = subprocess.run(['docker', 'stop', '-t', '3', 'robot-nav2'], timeout=15,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                shutdown_confirmed = shutdown.returncode == 0
            except (OSError, subprocess.TimeoutExpired):
                pass
            if process:
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.terminate()
            store.update(task_id, status='indeterminate', reason='worker_failed_navigation_disabled' if shutdown_confirmed else 'worker_failed_shutdown_unconfirmed',
                         running=False, stopped=False, navigation_shutdown_confirmed=shutdown_confirmed)
        from robot_skills.exploration_upgrade import publish_terminal
        try:
            publish_terminal(store,task_id)
        except Exception as error:
            # Keep the authoritative terminal result and its pending marker.
            # Memory mode cannot proceed until reconciliation succeeds.
            print('exploration_memory_pending:'+type(error).__name__,flush=True)


if __name__ == '__main__':
    main()
