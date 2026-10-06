"""Transport-neutral skills; mission-scoped investigation goals are checked locally."""
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
        state['operator_assistance_active'] = bool(self.store.meta('active_operator_assistance'))
        state['motion_tools_enabled'] = (self.investigations_enabled() and not state['stop_latched'] and not state['operator_assistance_active'] and
                                         self.store.meta('active_mission') in (None, self.mission_id))
        state['motion_scope'] = ('mission_scoped_investigation_goals' if self.investigations_enabled() else
                                 'status_and_stop_only')
        return state

    def catalog_view(self):
        catalog = self.store.meta('catalog', {})
        if time.time() > catalog.get('expires_unix_s', 0):
            return {'status': 'catalog_missing_or_expired', 'candidates': [],
                    'requires_refresh': True}
        return catalog


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
                'decision_options': (['start_investigation','view_map','plan_navigation','navigate_to_pose',
                                     'navigate_through_poses','probe_forward','recover_short_reverse','finish_investigation','cancel_task','stop_robot']
                                     if self.investigations_enabled() and self.store.meta('mission:'+self.mission_id,{}).get('phase')=='ai_investigation'
                                     else ['stop_robot']),
                'mission': self.store.meta('mission:'+self.mission_id) if self.mission_id else None,
                'operator_assistance': self.store.meta('last_operator_assistance'),
                'arbitrary_navigation_available': self.investigations_enabled(),
                'investigations':self.investigation_context(),
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

    def investigations_enabled(self):
        return bool(self.mission_id and self.store.meta('mission:'+self.mission_id,{}).get('two_stage'))

    def investigation_context(self):
        if not self.investigations_enabled(): return None
        m=self.store.meta('mission:'+self.mission_id,{})
        from .investigation import oscillating
        events=self.store.meta('investigation_memory:'+self.mission_id,[])
        return {'phase':m.get('phase'),'handoff_reason':(m.get('handoff') or {}).get('reason'),
                'active':self.store.meta('investigation:'+str(m.get('active_investigation'))),
                'history':[self.store.meta('investigation:'+iid) for iid in m.get('investigation_ids',[])][-12:],
                'failure_memory':events[-24:],'oscillation_detected':oscillating(events),
                'recovery_policy':m.get('recovery_policy'),
                'recovery_attempts':self.store.meta('recovery_attempts:'+self.mission_id,[]),
                'world_complete':False}

    def view_map(self, request_id):
        """Asynchronous online map/image snapshot; poll get_task_status."""
        return self._submit('view_map',{},request_id)

    def start_candidate_search(self, request_id):
        return self._submit('search_frontiers',{},request_id)

    def plan_navigation(self, poses, map_epoch, request_id):
        from .investigation import poses_payload
        return self._submit('plan_navigation',poses_payload(poses,map_epoch),request_id)

    def start_investigation(self, subject, hypothesis, task_type, request_id, passage=None):
        from .investigation import create
        return create(self.store,self.mission_id,subject,hypothesis,task_type,request_id,passage)

    def finish_investigation(self, investigation_id, outcome, assessment, evidence_task_ids):
        from .investigation import finish
        return finish(self.store,self.mission_id,investigation_id,outcome,assessment,evidence_task_ids)

    def navigate_to_pose(self, investigation_id, pose, map_epoch, request_id):
        from .investigation import poses_payload
        return self._submit('navigate_to_pose',{**poses_payload([pose],map_epoch),
                            'investigation_id':investigation_id},request_id)

    def navigate_through_poses(self, investigation_id, poses, map_epoch, request_id):
        from .investigation import poses_payload
        return self._submit('navigate_through_poses',{**poses_payload(poses,map_epoch),
                            'investigation_id':investigation_id},request_id)

    def probe_forward(self, investigation_id, map_epoch, request_id):
        return self._submit('probe_forward',{'investigation_id':investigation_id,'map_epoch':map_epoch},request_id)

    def recover_short_reverse(self, investigation_id, map_epoch, source_task_id, request_id):
        from .store import valid_uuid
        valid_uuid(source_task_id)
        return self._submit('recover_short_reverse',{'investigation_id':investigation_id,
                            'map_epoch':map_epoch,'source_task_id':source_task_id},request_id)

    def cancel_task(self, task_id):
        task=self.store.get(task_id)
        if not self.mission_id or task.get('mission_id')!=self.mission_id or task['kind']=='stop_robot':
            raise ValueError('cancel_requires_owned_non_stop_task')
        return self.store.cancel(task_id)


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
