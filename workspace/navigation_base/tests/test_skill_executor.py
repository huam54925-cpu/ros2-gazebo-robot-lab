"""Failure injection for the Robot Skills cancellation contract (ROS runtime)."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from robot_skills.ros_worker import Executor


class SkillCancelTests(unittest.TestCase):
    def worker(self):
        worker = object.__new__(Executor)
        worker.handle = Mock()
        worker.action_result = object()
        worker.wait = Mock(side_effect=[NS(goals_canceling=[1]), NS(status=5)])
        worker.event = Mock()
        worker.stop_confirmed = Mock(return_value=True)
        worker.pause_navigation = Mock(return_value=True)
        return worker

    def test_cancel_waits_for_terminal_result_then_stationary(self):
        worker = self.worker()
        self.assertTrue(worker.cancel_active())
        self.assertEqual(worker.wait.call_count, 2)
        worker.stop_confirmed.assert_called_once()
        worker.pause_navigation.assert_not_called()

    def test_cancel_timeout_and_pause_failure_cannot_claim_stop(self):
        worker = self.worker()
        worker.wait.side_effect = RuntimeError('timeout')
        worker.pause_navigation.return_value = False
        self.assertFalse(worker.cancel_active())
        worker.stop_confirmed.assert_not_called()

    def test_cancel_timeout_requires_pause_and_fresh_stop(self):
        worker = self.worker()
        worker.wait.side_effect = RuntimeError('timeout')
        worker.stop_confirmed.return_value = False
        self.assertFalse(worker.cancel_active())
        worker.pause_navigation.assert_called_once()
        worker.stop_confirmed.assert_called_once()

    def test_nonterminal_response_requires_pause(self):
        worker = self.worker()
        worker.wait.side_effect = [NS(goals_canceling=[]), NS(status=2)]
        self.assertTrue(worker.cancel_active())
        worker.pause_navigation.assert_called_once()


    def test_rotation_new_map_invalidates_remaining_sweep(self):
        worker = self.worker()
        worker.spin_client = Mock()
        handle = Mock(accepted=True)
        handle.get_result_async.return_value.done.return_value = False
        worker.wait = Mock(return_value=handle)
        worker.pose = Mock(side_effect=[[0,0,0],[0,0,.02]])
        worker.health = Mock(); worker.spin_once = Mock()
        worker.latest = {'map_version':'v1','nearest':3.0}
        worker.rotation_check = Mock(side_effect=[{'safe':True},{'safe':False,'reason':'body_sweep_nonfree'}])
        worker.store = Mock(); worker.identifier = 'test'
        import math
        with self.assertRaisesRegex(RuntimeError,'rotation_path_invalidated'):
            worker.perform_rotation({'signed_angle_rad':math.pi/2},{})
        self.assertEqual(worker.rotation_check.call_count,2)
        sent = worker.spin_client.send_goal_async.call_args.args[0]
        self.assertFalse(sent.disable_collision_checks)

    def test_queued_tf_is_drained_without_relaxing_age_limits(self):
        worker=object.__new__(Executor); worker.node=Mock(); worker.task=None
        with patch('robot_skills.ros_worker.Explorer.health',side_effect=[RuntimeError('localization_stale'),None]) as health, \
                patch('robot_skills.ros_worker.rclpy.spin_once') as spin:
            worker.health()
            self.assertEqual(health.call_count,2)
            self.assertEqual(spin.call_count,128)
            self.assertEqual(spin.call_args.kwargs,{'timeout_sec':0.})

    def test_genuinely_stale_tf_still_fails_after_bounded_drain(self):
        worker=object.__new__(Executor); worker.node=Mock(); worker.task=None
        with patch('robot_skills.ros_worker.Explorer.health',side_effect=RuntimeError('localization_stale')) as health, \
                patch('robot_skills.ros_worker.rclpy.spin_once'):
            with self.assertRaisesRegex(RuntimeError,'localization_stale'): worker.health()
            self.assertEqual(health.call_count,2)

if __name__ == '__main__':
    unittest.main()
