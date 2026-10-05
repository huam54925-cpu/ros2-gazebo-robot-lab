import sys,unittest
from pathlib import Path
from unittest.mock import patch
W=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(W),str(W/'navigation_base/exploration')]
from robot_skills.operator_assist import Assist,Explorer

class OperatorFeedbackTests(unittest.TestCase):
    def test_queued_tf_drained_once_without_relaxing_age_check(self):
        worker=Assist.__new__(Assist);worker.node=object()
        with patch.object(Explorer,'health',side_effect=[RuntimeError('localization_stale'),None]) as check, patch('robot_skills.operator_assist.rclpy.spin_once') as spin:
            worker.health();self.assertEqual(check.call_count,2);self.assertEqual(spin.call_count,128)
    def test_stale_tf_after_drain_still_fails(self):
        worker=Assist.__new__(Assist);worker.node=object()
        with patch.object(Explorer,'health',side_effect=RuntimeError('localization_stale')),patch('robot_skills.operator_assist.rclpy.spin_once'):
            with self.assertRaisesRegex(RuntimeError,'localization_stale'):worker.health()
    def test_stale_scan_is_not_reclassified_as_tf_queue_delay(self):
        worker=Assist.__new__(Assist);worker.node=object()
        with patch.object(Explorer,'health',side_effect=RuntimeError('scan_at_stale')),patch('robot_skills.operator_assist.rclpy.spin_once') as spin:
            with self.assertRaisesRegex(RuntimeError,'scan_at_stale'):worker.health()
            spin.assert_not_called()
