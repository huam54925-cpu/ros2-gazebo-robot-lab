"""Recompute navigation metrics and synchronous timing from persisted evaluation traces."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def close(a, b):
    assert math.isclose(a, b, abs_tol=1e-8, rel_tol=1e-10), (a, b)


def verify(root):
    root = Path(root)
    report = json.loads((root / 'evaluation.json').read_text())
    assert report['status'] == 'completed'
    assert report['checkpoint_unchanged']
    assert report['checkpoint_hashes_before'] == report['checkpoint_hashes_after']
    assert len(report['episodes']) == len(report['seeds'])
    verified_steps = 0
    for episode in report['episodes']:
        rows = [json.loads(line) for line in (root / episode['trace']).read_text().splitlines()]
        assert rows[0]['event'] == 'reset'
        assert episode['status'] == 'completed'
        assert len(rows) - 1 == episode['steps']
        path = reward = 0.
        for before, current in zip(rows, rows[1:]):
            assert current['event'] == 'step'
            assert current['episode_id'] == before['episode_id']
            assert current['action_id'] == before['action_id'] + 1
            close(current['sim_time'] - before['sim_time'], .2)
            for stamp in current['sensor_times'].values():
                assert abs(stamp - current['sim_time']) <= 1e-6
            ack = current['ack']
            assert ack['steps'] == 200
            assert ack['applied_ns'] == round(before['sim_time'] * 1e9)
            assert ack['first_ns'] == ack['applied_ns'] + 1_000_000
            assert ack['last_ns'] == round(current['sim_time'] * 1e9)
            path += math.dist(before['truth_pose'][:2], current['truth_pose'][:2])
            reward += current['reward']
        close(path, episode['path_length_m'])
        close(reward, episode['episode_return'])
        close(rows[-1]['sim_time'] - rows[0]['sim_time'], episode['sim_elapsed_s'])
        close(rows[-1]['truth_distance_m'], episode['final_distance_m'])
        close(min(row['truth_distance_m'] for row in rows), episode['minimum_distance_m'])
        assert sum(row.get('stalled', False) for row in rows) == episode['stalled_steps']
        final = rows[-1]
        success = final.get('terminated') and final['truth_distance_m'] <= report['arrival_threshold_m']
        timeout = final.get('truncated') and not final.get('terminated')
        assert (episode['outcome'] == 'success') == bool(success)
        assert (episode['outcome'] == 'timeout') == bool(timeout)
        if timeout:
            close(episode['sim_elapsed_s'], report['environment_time_limit_s'])
        verified_steps += episode['steps']
    episodes = report['episodes']
    n = len(episodes)
    metrics = report['metrics']
    close(metrics['success_rate'], sum(e['outcome'] == 'success' for e in episodes) / n)
    close(metrics['timeout_rate'], sum(e['outcome'] == 'timeout' for e in episodes) / n)
    for metric, field in [('mean_path_length_m', 'path_length_m'), ('mean_episode_return', 'episode_return'),
                          ('mean_sim_elapsed_s', 'sim_elapsed_s')]:
        close(metrics[metric], sum(e[field] for e in episodes) / n)
    return {'status': 'passed', 'episodes': n, 'verified_transitions': verified_steps,
            'checks': ['trajectory path', 'reward sums', 'distance extrema', 'stall counts',
                       'episode/action identity', '200 physical steps and ACK times',
                       'sensor timing', 'outcomes and aggregate metrics', 'checkpoint unchanged'],
            'sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in sorted(root.iterdir()) if p.suffix in ('.jsonl', '.json')}}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('evaluation_directory', type=Path)
    args = parser.parse_args()
    print(json.dumps(verify(args.evaluation_directory), indent=2))
