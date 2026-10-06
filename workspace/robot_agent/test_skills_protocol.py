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
from unittest.mock import patch, Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_skills.api import RobotSkills
from robot_skills.store import Store
from mcp_skills_server import registered_tools
from pydantic import ValidationError


class ProtocolTests(unittest.TestCase):
    def mission_tools(self):
        import importlib.util
        spec=importlib.util.spec_from_file_location('mission_protocol_under_test',Path(__file__).with_name('mcp_skills_server.py'))
        module=importlib.util.module_from_spec(spec)
        skills=Mock();skills.investigations_enabled.return_value=True
        with patch('robot_skills.api.RobotSkills',return_value=skills):spec.loader.exec_module(module)
        return {t.name:t for t in module.registered_tools},skills

    def test_map_pose_schema_requires_yaw_and_rejects_extra_fields(self):
        tools,_=self.mission_tools()
        for name,field in [('start_investigation','subject'),('navigate_to_pose','pose'),
                           ('plan_navigation','poses'),('navigate_through_poses','poses')]:
            schema=tools[name].parameters
            pose=schema['$defs']['MapPose']
            self.assertEqual(set(pose['required']),{'x','y','yaw'})
            self.assertFalse(pose['additionalProperties'])
        model=tools['start_investigation'].fn_metadata.arg_model
        args={'subject':{'x':1.,'y':2.},'hypothesis':'unknown','task_type':'observe_structure','request_id':'id'}
        with self.assertRaises(ValidationError):model.model_validate(args)
        args['subject']['yaw']=0.
        self.assertEqual(model.model_validate(args).subject,args['subject'])
        args['subject']['speed']=1.
        with self.assertRaises(ValidationError):model.model_validate(args)

    def test_local_rule_error_is_returned_instead_of_hidden_sdk_crash(self):
        tools,skills=self.mission_tools()
        skills.start_investigation.side_effect=ValueError('finish_current_investigation_first')
        result=tools['start_investigation'].fn(subject={'x':1.,'y':2.,'yaw':0.},
            hypothesis='unknown',task_type='observe_structure',request_id='id')
        self.assertEqual(result,{'status':'request_rejected','reason':'finish_current_investigation_first'})

    def test_outside_mission_only_status_and_stop_are_registered(self):
        self.assertEqual({t.name for t in registered_tools},
                         {'get_robot_state','get_decision_context','get_task_status','stop_robot'})
        for tool in registered_tools:self.assertFalse(tool.parameters['additionalProperties'])

    def test_old_motion_skills_are_rejected_before_launch(self):
        import uuid
        with tempfile.TemporaryDirectory() as directory:
            store=Store(directory)
            for kind in ('fixed_step','perform_observation'):
                with self.assertRaisesRegex(ValueError,'retired_skill'):
                    store.submit(kind,{},str(uuid.uuid4()))
            task,launch=store.submit('execute_frontier',{'frontier_id':'old'},str(uuid.uuid4()))
            self.assertFalse(launch)
            self.assertEqual(task['reason'],'two_stage_mission_required')


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




if __name__ == '__main__':
    unittest.main()
