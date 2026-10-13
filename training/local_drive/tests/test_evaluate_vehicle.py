import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from training.checkpoints import digest
from training.envs.contracts import InfrastructureError, SCHEMA_VERSION
from training.evaluate_vehicle import checkpoint_for_evaluation, run_episode, summarize


def frame(x=0., y=0., distance=3., step=0):
    return SimpleNamespace(episode_id=1, action_id=step, sim_time=900. + step * .2,
                           sensor_times={}, odom_pose=(999., 999., 0.), ack={},
                           evaluation={'truth_pose': (x, y, 0.), 'truth_dist': distance, 'roll': 0., 'pitch': 0.})


class Policy:
    def predict(self, obs, deterministic):
        assert deterministic
        return [.25, -.5], None


class Environment:
    def __init__(self, transitions, close_error=None):
        self.transitions = iter(transitions)
        self.backend = SimpleNamespace(cleanup_errors=[], name=None)
        self.close_error = close_error
        self.closed = False

    def reset(self, seed):
        self.last_frame = frame()
        return [0.], {'backend_seed': seed + 1}

    def step(self, action):
        item = next(self.transitions)
        if isinstance(item, Exception):
            raise item
        self.last_frame, reward, terminated, truncated = item
        return [0.], reward, terminated, truncated, {'stalled': False}

    def close(self):
        self.closed = True
        if self.close_error:
            self.backend.cleanup_errors.append(self.close_error)


class EvaluationTests(unittest.TestCase):
    def run_trial(self, env, steps=600, interrupted=lambda: False):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'trace.jsonl'
            result = run_episode(Policy(), env, 7, steps, path, interrupted)
            records = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertTrue(env.closed)
        return result, records

    def test_truth_path_and_undiscounted_reward_include_terminal_step(self):
        env = Environment([(frame(3, 4, 2., 1), 2., False, False),
                           (frame(6, 8, .3, 2), 10., True, False)])
        result, records = self.run_trial(env)
        self.assertEqual(result['outcome'], 'success')
        self.assertEqual(result['path_length_m'], 10.)
        self.assertEqual(result['episode_return'], 12.)
        self.assertAlmostEqual(result['sim_elapsed_s'], .4)
        self.assertEqual(len(records), 3)

    def test_timeout_is_environment_truncation(self):
        result, _ = self.run_trial(Environment([(frame(step=600), -6., False, True)]))
        self.assertEqual(result['outcome'], 'timeout')

    def test_evaluator_cap_is_not_timeout(self):
        result, _ = self.run_trial(Environment([(frame(step=1), -.1, False, False)]), steps=1)
        self.assertEqual(result['outcome'], 'evaluation_limit')
        self.assertEqual(summarize([result], 1)['timeout_rate'], 0.)

    def test_arrival_at_deadline_wins_over_truncation(self):
        result, _ = self.run_trial(Environment([(frame(distance=.35, step=600), 10., True, True)]))
        self.assertEqual(result['outcome'], 'success')

    def test_other_terminal_is_not_arrival(self):
        result, _ = self.run_trial(Environment([(frame(distance=2., step=1), -.1, True, False)]))
        self.assertEqual(result['outcome'], 'terminal_failure')

    def test_failed_transition_is_not_counted_and_partial_trace_survives(self):
        env = Environment([(frame(1, 0, 2., 1), 2., False, False), InfrastructureError('missing ACK')])
        result, records = self.run_trial(env)
        self.assertEqual(result['status'], 'failed')
        self.assertEqual(result['outcome'], 'infrastructure_error')
        self.assertEqual(result['steps'], 1)
        self.assertEqual(result['path_length_m'], 1.)
        self.assertEqual(len(records), 2)
        self.assertIsNone(summarize([result], 3)['success_rate'])
        self.assertIsNone(summarize([result], 3)['timeout_rate'])

    def test_cleanup_failure_invalidates_completed_result(self):
        result, _ = self.run_trial(Environment([(frame(distance=.1, step=1), 10., True, False)], 'stop failed'))
        self.assertEqual(result['observed_outcome'], 'success')
        self.assertEqual(result['outcome'], 'cleanup_error')
        self.assertIsNone(summarize([result], 1)['success_rate'])

    def test_interrupt_preserves_reset_and_cleans_up(self):
        result, records = self.run_trial(Environment([]), interrupted=lambda: True)
        self.assertEqual(result['outcome'], 'interrupted')
        self.assertEqual(result['steps'], 0)
        self.assertEqual(len(records), 1)

    def test_complete_metrics_and_incomplete_denominator(self):
        success = {'status': 'completed', 'outcome': 'success', 'path_length_m': 3., 'episode_return': 12., 'sim_elapsed_s': 10.}
        timeout = {**success, 'outcome': 'timeout', 'path_length_m': 5., 'episode_return': -4., 'sim_elapsed_s': 120.}
        result = summarize([success, timeout], 2)
        self.assertEqual(result['success_rate'], .5)
        self.assertEqual(result['timeout_rate'], .5)
        self.assertEqual(result['mean_path_length_m'], 4.)
        self.assertEqual(result['mean_episode_return'], 4.)
        self.assertIsNone(summarize([success], 2)['success_rate'])

    def test_checkpoint_pointer_direct_directory_and_corruption(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            checkpoint = root / 'step-72-test'
            checkpoint.mkdir()
            for name in ('model.zip', 'replay.pkl'):
                (checkpoint / name).write_bytes(b'Never deserialize these test bytes')
            manifest = {'environment': 'LocalDriveEnv', 'schema': SCHEMA_VERSION,
                        'files': {name: digest(checkpoint / name) for name in ('model.zip', 'replay.pkl')}}
            (checkpoint / 'manifest.json').write_text(json.dumps(manifest))
            (root / 'latest.txt').write_text(checkpoint.name)
            self.assertEqual(checkpoint_for_evaluation(root)[0], checkpoint)
            self.assertEqual(checkpoint_for_evaluation(checkpoint)[0], checkpoint)
            (checkpoint / 'replay.pkl').write_bytes(b'corrupt')
            with self.assertRaisesRegex(ValueError, 'Corrupt checkpoint'):
                checkpoint_for_evaluation(checkpoint)


if __name__ == '__main__':
    unittest.main()
