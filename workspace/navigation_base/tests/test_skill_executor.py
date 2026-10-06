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


    def test_waits_for_lifecycle_activation_and_honors_cancel(self):
        worker=self.worker();worker.task=None;worker.node=Mock();worker.spin_once=Mock()
        worker.node.create_client.return_value.wait_for_service.return_value=True
        worker.wait=Mock(side_effect=[NS(current_state=NS(id=i)) for i in (2,3,3,3,3)])
        worker.wait_navigation_active()
        self.assertEqual(worker.wait.call_count,5)
        self.assertEqual(worker.node.destroy_client.call_count,4)
        worker.task={'task_id':'active'};worker.identifier='active';worker.store=Mock()
        worker.store.should_cancel.return_value=True
        with self.assertRaisesRegex(RuntimeError,'stop_requested'):worker.wait_navigation_active()

    def test_sensor_offset_survives_legacy_observation_removal(self):
        worker=self.worker()
        worker.buffer=Mock()
        worker.buffer.lookup_transform.return_value=NS(transform=NS(translation=NS(x=-.7057095,y=0.)))
        self.assertEqual(worker.sensor_offset(),(-.7057095,0.))
        worker.buffer.lookup_transform.side_effect=RuntimeError('no TF')
        with self.assertRaisesRegex(RuntimeError,'lidar_transform_unavailable'):
            worker.sensor_offset()

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
