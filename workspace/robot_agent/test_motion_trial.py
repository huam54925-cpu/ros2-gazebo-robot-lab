import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import motion_trial as motion

ID = '10ddc565-2521-4d52-8d00-eb65f56bead0'


class MotionTrialTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.patch = patch.object(motion, 'DIRECTORY', self.directory)
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def test_uuid_rejects_command_or_path(self):
        for value in ['../test', 'x; touch file', '', ID.upper()]:
            with self.assertRaises(ValueError):
                motion.identifier(value)

    def test_completed_request_is_replayed_without_subprocess(self):
        (self.directory / (ID + '.request')).write_text('{}')
        (self.directory / (ID + '.result.json')).write_text(json.dumps({'status': 'succeeded', 'stopped': True}))
        with patch.object(motion, '_skills') as launch:
            result = motion.move_forward_trial(ID)
        launch.assert_not_called()
        self.assertTrue(result['replayed_without_movement'])

    def test_unresolved_request_blocks_new_movement(self):
        (self.directory / (ID + '.request')).write_text('{}')
        with patch.object(motion, '_skills') as launch:
            result = motion.move_forward_trial('ba0e3efc-272d-4453-92fb-6ea1cfdc8a6d')
        launch.assert_not_called()
        self.assertEqual(result['reason'], 'unresolved_previous_trial')

    def test_cancel_acknowledgement_does_not_claim_stopped(self):
        (self.directory / (ID + '.request')).write_text('{}')
        result = motion.cancel_motion_trial(ID)
        self.assertTrue(result['cancel_requested'])
        self.assertFalse(result['stopped'])
        self.assertTrue((self.directory / (ID + '.cancel')).exists())

    def test_cancel_unknown_id_does_not_create_files(self):
        self.assertEqual(motion.cancel_motion_trial(ID)['status'], 'unknown_request')
        self.assertEqual(list(self.directory.iterdir()), [])

    def test_unconfirmed_stop_blocks_new_movement(self):
        previous = self.directory / 'previous.result.json'
        previous.write_text(json.dumps({'status': 'stop_unconfirmed'}))
        with patch.object(motion, '_skills') as launch:
            result = motion.move_forward_trial(ID)
        launch.assert_not_called()
        self.assertEqual(result['reason'], 'previous_stop_unconfirmed')


if __name__ == '__main__':
    unittest.main()
