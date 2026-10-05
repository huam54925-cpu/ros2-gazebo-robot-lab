"""Transport-neutral skill API; model requests contain IDs only."""
import json
from pathlib import Path
import subprocess
import sys
import time

from .store import Store, ACTIVE, DIRECTORY

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / 'robot_agent'))
from robot_status import get_robot_status


def robot_command(operation, task_id=None):
    args = ['docker', 'exec', 'robot-nav2', 'bash', '-lc',
            'source /opt/ros/lyrical/setup.bash; exec python3 /work/robot_ws/robot_skills/ros_worker.py "$@"',
            'bash', operation]
    return args + ([task_id] if task_id else [])


class RobotSkills:
    def __init__(self, store=None, mission_id=None, observations=False):
        self.store = store or Store()
        self.mission_id, self.observations = mission_id, observations

    def get_robot_state(self):
        state = get_robot_status()
        state['stop_latched'] = self.store.meta('stop_latched', False)
        state['motion_tools_enabled'] = (not state['stop_latched'] and
                                         self.store.meta('active_mission') in (None, self.mission_id))
        state['motion_scope'] = 'validated_frontier_and_observation_ids' if self.observations else 'validated_frontier_ids_only'
        return state

    def catalog_view(self):
        catalog = self.store.meta('catalog', {})
        if time.time() > catalog.get('expires_unix_s', 0):
            return {'status': 'catalog_missing_or_expired', 'candidates': [],
                    'requires_refresh': True}
        return catalog

    def get_safe_frontiers(self):
        state = self.get_robot_state()
        current_version = state.get('sources', {}).get('map', {}).get('map_version')
        catalog = self.catalog_view()
        from .exploration_upgrade import mode_for, search_for, history_revision, path_clearance_for
        mode=mode_for(self.store,self.mission_id)
        audit=catalog.get('exploration_upgrade',{})
        compatible=(catalog.get('path_clearance_m',2.15)==path_clearance_for(self.store,self.mission_id) and
            catalog.get('candidate_search','baseline')==search_for(self.store,self.mission_id) and
            audit.get('mode','baseline')==mode and (mode=='baseline' or
            (audit.get('mission_id')==self.mission_id and audit.get('history_revision')==history_revision(self.store.tasks(),self.mission_id))))
        if (catalog.get('candidates') and catalog.get('catalog_schema') == 3 and catalog.get('map_version') == current_version and not catalog.get('requires_refresh')
                and compatible and catalog.get('expires_unix_s', 0)-time.time() > 90):
            return catalog
        if any(t['status'] in ACTIVE for t in self.store.tasks()):
            return {'status': 'task_active', 'candidates': [], 'requires_refresh': True}
        if state['status'] != 'available' or state['stop_latched']:
            return {'status': 'robot_unavailable_or_stop_latched', 'candidates': []}
        # Planning only. No navigation action or velocity publisher is used here.
        with (self.store.directory / 'catalog-worker.log').open('w') as log:
            try:
                result = subprocess.run(robot_command('catalog'), stdout=log, stderr=log, timeout=95)
            except subprocess.TimeoutExpired:
                return {'status': 'catalog_timeout', 'candidates': []}
        if result.returncode:
            return {'status': 'catalog_unavailable', 'candidates': [],
                    'reason': 'check_catalog_worker_log'}
        return self.catalog_view()

    def get_observation_options(self):
        if not self.observations:
            return {'status': 'observations_not_enabled', 'options': []}
        state = self.get_robot_state()
        if state['status'] != 'available' or state['stop_latched'] or any(t['status'] in ACTIVE for t in self.store.tasks()):
            return {'status': 'robot_unavailable_or_busy', 'options': []}
        catalog = self.store.meta('observation_catalog', {})
        version = state.get('sources', {}).get('map', {}).get('map_version')
        if catalog.get('expires_unix_s', 0)-time.time() > 90 and catalog.get('map_version') == version:
            return catalog
        with (self.store.directory / 'observation-catalog-worker.log').open('w') as log:
            try:
                result = subprocess.run(robot_command('observations'), stdout=log, stderr=log, timeout=95)
            except subprocess.TimeoutExpired:
                return {'status': 'observation_catalog_timeout', 'options': []}
        return self.store.meta('observation_catalog', {}) if result.returncode == 0 else {'status': 'observation_catalog_unavailable', 'options': []}

    def perform_observation(self, option_id, request_id):
        if not self.observations:
            raise ValueError('observations_not_enabled')
        if not isinstance(option_id, str) or len(option_id) > 64:
            raise ValueError('invalid_observation_id')
        return self._submit('perform_observation', {'option_id': option_id}, request_id)

    def get_decision_context(self):
        """Reads only; does not refresh/plan, launch workers, resume or move."""
        state = self.get_robot_state()
        tasks = self.store.tasks()
        catalog = self.catalog_view()
        version = state.get('sources', {}).get('map', {}).get('map_version')
        from .regional import context as regional_context
        return {'robot': state, 'frontiers': catalog,
                'exploration_upgrade':catalog.get('exploration_upgrade',{'mode':'baseline'}),
                'exploration': regional_context(catalog, tasks, self.mission_id),
                'map_version_changed_since_candidates': version != catalog.get('map_version'),
                'active_tasks': [t for t in tasks if t['status'] in ACTIVE],
                'recent_tasks': [t for t in tasks if t.get('mission_id') == self.mission_id][:5] if self.mission_id else tasks[:5],
                'map_session_task_history':[
                    {'task_id':t['task_id'],'status':t['status'],'reason':t.get('reason'),'stopped':t.get('stopped'),
                     'goal':{k:(t.get('candidate') or {}).get(k) for k in ('x','y','yaw','frontier_id')},
                     'mission_id':t.get('mission_id')}
                    for t in tasks if t.get('candidate') and catalog.get('region_epoch')
                    and t['candidate'].get('region_epoch')==catalog['region_epoch']
                    and t.get('mission_id')!=self.mission_id][:5],
                'decision_options': ['execute_frontier', 'stop_robot'] + (['perform_observation'] if self.observations else []),
                'observation_options': self.store.meta('observation_catalog', {}) if self.observations else {'options': []},
                'mission': self.store.meta('mission:'+self.mission_id) if self.mission_id else None,
                'observe_available': self.observations, 'arbitrary_navigation_available': False,
                'authority': 'Only local task state establishes success, cancellation or stopped.'}

    def get_task_status(self, task_id):
        task = self.store.get(task_id)
        task['report_age_wall_s'] = max(0, time.time()-task['updated_unix_s'])
        return task

    def _submit(self, kind, payload, request_id):
        task, launch = self.store.submit(kind, payload, request_id, self.mission_id)
        if launch:
            path = self.store.directory / (task['task_id'] + '.supervisor.log')
            try:
                with path.open('w') as log:
                    subprocess.Popen([sys.executable, str(WORKSPACE / 'robot_skills/supervisor.py'),
                                      task['task_id']], stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                     start_new_session=True, close_fds=True)
            except OSError:
                task = self.store.update(task['task_id'], status='rejected', reason='supervisor_start_failed')
        return task

    def execute_frontier(self, frontier_id, request_id):
        if not isinstance(frontier_id, str) or len(frontier_id) > 64:
            raise ValueError('invalid_frontier_id')
        return self._submit('execute_frontier', {'frontier_id': frontier_id}, request_id)

    def fixed_step(self, request_id):
        return self._submit('fixed_step', {}, request_id)

    def stop_robot(self, request_id):
        return self._submit('stop_robot', {}, request_id)

    def resume_operator_only(self):
        result = subprocess.run(robot_command('resume'), capture_output=True, text=True, timeout=45)
        if result.returncode:
            return {'status': 'resume_failed', 'stop_latched': self.store.meta('stop_latched', False)}
        return {'status': 'resumed', 'stop_latched': self.store.meta('stop_latched', False)}

    def recover_operator_only(self, task_id):
        self.store.get(task_id)
        inspection = subprocess.run(['docker','ps','--format','{{.Names}}'],
                                    capture_output=True, text=True, timeout=5)
        if inspection.returncode != 0:
            raise ValueError('cannot_verify_nav2_shutdown')
        if 'robot-nav2' in inspection.stdout.splitlines():
            raise ValueError('recovery_requires_nav2_shutdown')
        observations = []
        for _ in range(4):
            state = self.get_robot_state()
            sources = state.get('sources', {})
            odom, guard = sources.get('odometry', {}), sources.get('guard_output', {})
            if state['status'] != 'available' or not odom.get('fresh') or not guard.get('fresh'):
                raise ValueError('stop_feedback_unavailable')
            velocity = odom.get('velocity', {})
            if len(velocity) != 2 or any(abs(v) >= .02 for v in velocity.values()):
                raise ValueError('robot_not_stationary')
            if abs(guard.get('linear_x_m_s', 1)) > 1e-6 or abs(guard.get('angular_z_rad_s', 1)) > 1e-6:
                raise ValueError('guard_output_not_zero')
            observations.append({'stamp_sim_s': odom['stamp_sim_s'], 'velocity': velocity})
            time.sleep(.4)
        if observations[-1]['stamp_sim_s'] <= observations[0]['stamp_sim_s']:
            raise ValueError('no_new_odometry_during_recovery')
        return self.store.record_stop_recovery(task_id, {'nav2_not_running': True,
                                                       'fresh_stationary_observations': observations})
