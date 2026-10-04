import asyncio
import json
import os
import subprocess
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_skills.api import RobotSkills
from robot_skills.store import Store
from mcp_skills_server import registered_tools
from pydantic import ValidationError


class ProtocolTests(unittest.TestCase):
    def test_exact_six_tools_and_no_extra_fields(self):
        names = {t.name for t in registered_tools}
        self.assertEqual(names, {'get_robot_state','get_decision_context','get_safe_frontiers',
                                 'execute_frontier','get_task_status','stop_robot'})
        for tool in registered_tools:
            self.assertFalse(tool.parameters['additionalProperties'])
        execute = next(t for t in registered_tools if t.name == 'execute_frontier')
        self.assertEqual(set(execute.parameters['properties']), {'frontier_id','request_id'})
        for extra in ('x','y','yaw','speed','clearance','command'):
            with self.assertRaises(ValidationError):
                execute.fn_metadata.validate_arguments({'frontier_id':'F_1','request_id':'uuid',extra:0})

    def test_decision_context_does_not_plan_or_launch(self):
        with tempfile.TemporaryDirectory() as directory:
            skills = RobotSkills(Store(directory))
            with patch('robot_skills.api.get_robot_status', return_value={'status':'unavailable'}), \
                    patch('robot_skills.api.subprocess.Popen') as launch, \
                    patch('robot_skills.api.subprocess.run') as command:
                context = skills.get_decision_context()
            launch.assert_not_called(); command.assert_not_called()
            self.assertEqual(context['frontiers']['candidates'], [])
            self.assertFalse(context['arbitrary_navigation_available'])


    def test_recovery_requires_verified_nav2_shutdown(self):
        with tempfile.TemporaryDirectory() as directory:
            skills = RobotSkills(Store(directory))
            with patch.object(skills.store, 'get', return_value={}), \
                    patch('robot_skills.api.subprocess.run') as command, \
                    patch.object(skills, 'get_robot_state') as state:
                for result, expected in [(SimpleNamespace(returncode=1, stdout=''), 'cannot_verify'),
                                         (SimpleNamespace(returncode=0, stdout='robot-nav2\n'), 'shutdown')]:
                    command.return_value = result
                    with self.assertRaisesRegex(ValueError, expected):
                        skills.recover_operator_only('unused')
                state.assert_not_called()

    def test_recovery_requires_fresh_stationary_feedback(self):
        with tempfile.TemporaryDirectory() as directory:
            skills = RobotSkills(Store(directory))
            with patch.object(skills.store, 'get', return_value={}), \
                    patch('robot_skills.api.subprocess.run', return_value=SimpleNamespace(returncode=0, stdout='')), \
                    patch.object(skills, 'get_robot_state') as state, \
                    patch.object(skills.store, 'record_stop_recovery') as record:
                state.return_value = {'status': 'unavailable'}
                with self.assertRaisesRegex(ValueError, 'feedback_unavailable'):
                    skills.recover_operator_only('unused')
                state.return_value = {'status': 'available', 'sources': {
                    'odometry': {'fresh': True, 'velocity': {'linear_x_m_s': .1, 'angular_z_rad_s': 0}},
                    'guard_output': {'fresh': True}}}
                with self.assertRaisesRegex(ValueError, 'not_stationary'):
                    skills.recover_operator_only('unused')
                record.assert_not_called()


    def test_optional_observation_tools_accept_only_ids(self):
        script = """
import json
from mcp_skills_server import registered_tools
from pydantic import ValidationError
assert len(registered_tools)==8
observe=next(t for t in registered_tools if t.name=='perform_observation')
assert set(observe.parameters['properties'])=={'option_id','request_id'}
for field in ('angle','x','y','speed','clearance','resume'):
    try:
        observe.fn_metadata.validate_arguments({'option_id':'OBS','request_id':'UUID',field:1})
    except ValidationError: pass
    else: raise AssertionError(field)
print('passed')
"""
        result = subprocess.run([sys.executable,'-c',script], cwd=Path(__file__).resolve().parent,
            env={**os.environ,'ROBOT_OBSERVATIONS_ENABLED':'1'},capture_output=True,text=True,timeout=20)
        self.assertEqual(result.returncode,0,result.stderr)

    def test_near_expiry_catalog_is_refreshed_before_model_choice(self):
        with tempfile.TemporaryDirectory() as directory:
            skills=RobotSkills(Store(directory))
            skills.store.set_meta('catalog',{'map_version':'v','expires_unix_s':time.time()+45,'candidates':[]})
            state={'status':'available','stop_latched':False,'sources':{'map':{'map_version':'v'}}}
            with patch.object(skills,'get_robot_state',return_value=state), \
                    patch('robot_skills.api.subprocess.run',return_value=SimpleNamespace(returncode=0)) as plan:
                skills.get_safe_frontiers()
            plan.assert_called_once()

if __name__ == '__main__':
    unittest.main()
