"""Detached local task supervision survives an MCP client disconnect."""
import fcntl
from pathlib import Path
import subprocess
import sys

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
        process = None
        fallback = False
        try:
            with (store.directory / (task_id + '.ros.log')).open('w') as log:
                process = subprocess.Popen(robot_command('task', task_id), stdout=log, stderr=log)
                process.wait(timeout=65 if task['kind'] == 'stop_robot' else 290)
            final = store.get(task_id)
            fallback = process.returncode != 0 or final['status'] in ACTIVE or final['status'] == 'stop_unconfirmed'
        except (OSError, subprocess.TimeoutExpired):
            store.cancel(task_id)
            fallback = True
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
