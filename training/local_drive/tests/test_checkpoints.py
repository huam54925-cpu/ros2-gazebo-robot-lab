import tempfile
import unittest
from pathlib import Path
from training.checkpoints import save_checkpoint, verify_checkpoint

class Model:
    num_timesteps=12
    def save(self,p):Path(p).write_bytes(b'model')
    def save_replay_buffer(self,p):Path(p).write_bytes(b'replay')
class Broken(Model):
    def save_replay_buffer(self,p):raise OSError('simulated interruption')
class CheckpointTests(unittest.TestCase):
    def test_interruption_keeps_previous(self):
        with tempfile.TemporaryDirectory() as root:
            first=save_checkpoint(Model(),root,{})
            with self.assertRaises(OSError):save_checkpoint(Broken(),root,{})
            self.assertEqual(verify_checkpoint(root)[0],first)
    def test_corruption_detected(self):
        with tempfile.TemporaryDirectory() as root:
            first=save_checkpoint(Model(),root,{})
            (first/'replay.pkl').write_bytes(b'broken')
            with self.assertRaises(ValueError):verify_checkpoint(root)
