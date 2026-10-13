"""Read-only deterministic navigation evaluation; never learn or load replay data."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import subprocess
import time

from .checkpoints import digest, verify_checkpoint
from .envs.contracts import InfrastructureError, SCHEMA_VERSION


def checkpoint_for_evaluation(root):
    """Accept a checkpoint root/latest pointer or one immutable checkpoint directory."""
    root = Path(root)
    if (root / 'manifest.json').is_file():
        path = root
        manifest = json.loads((path / 'manifest.json').read_text())
        for name in ('model.zip', 'replay.pkl'):
            if digest(path / name) != manifest['files'][name]:
                raise ValueError('Corrupt checkpoint: ' + name)
    else:
        path, manifest = verify_checkpoint(root)
    if manifest.get('schema') != SCHEMA_VERSION or manifest.get('environment') != 'LocalDriveEnv':
        raise ValueError('Expected a compatible LocalDriveEnv checkpoint')
    return path, manifest


def write_json(path, value):
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def frame_record(frame):
    return {
        'episode_id': frame.episode_id, 'action_id': frame.action_id,
        'sim_time': frame.sim_time, 'sensor_times': frame.sensor_times,
        'truth_pose': list(frame.evaluation['truth_pose']),
        'truth_distance_m': float(frame.evaluation['truth_dist']),
        'odom_pose': list(frame.odom_pose),
        'roll': float(frame.evaluation['roll']), 'pitch': float(frame.evaluation['pitch']),
        'ack': frame.ack,
    }


def run_episode(model, env, seed, max_steps, trace_path, interrupted=lambda: False):
    """Count only successfully returned transitions; preserve partial failure evidence."""
    started = time.monotonic()
    result = {'seed': seed, 'status': 'incomplete', 'outcome': None, 'steps': 0,
              'episode_return': 0., 'path_length_m': 0., 'sim_elapsed_s': 0.,
              'initial_distance_m': None, 'final_distance_m': None,
              'minimum_distance_m': None, 'stalled_steps': 0, 'cleanup_errors': []}
    try:
        with Path(trace_path).open('x') as trace:
            obs, info = env.reset(seed=seed)
            initial = env.last_frame
            previous = initial.evaluation['truth_pose']
            result.update(backend_seed=info['backend_seed'], initial_distance_m=initial.evaluation['truth_dist'],
                          final_distance_m=initial.evaluation['truth_dist'], minimum_distance_m=initial.evaluation['truth_dist'])
            trace.write(json.dumps({'event': 'reset', **frame_record(initial)}, allow_nan=False) + '\n')
            trace.flush()
            for step in range(1, max_steps + 1):
                if interrupted():
                    result.update(outcome='interrupted', status='interrupted')
                    break
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = env.step(action)
                frame = env.last_frame
                pose = frame.evaluation['truth_pose']
                distance = float(frame.evaluation['truth_dist'])
                result['path_length_m'] += math.hypot(pose[0] - previous[0], pose[1] - previous[1])
                result['episode_return'] += float(reward)
                result['steps'] = step
                result['sim_elapsed_s'] = frame.sim_time - initial.sim_time
                result['final_distance_m'] = distance
                result['minimum_distance_m'] = min(result['minimum_distance_m'], distance)
                result['stalled_steps'] += int(info.get('stalled', False))
                trace.write(json.dumps({'event': 'step', **frame_record(frame),
                                       'action': [float(x) for x in action], 'reward': float(reward),
                                       'terminated': bool(terminated), 'truncated': bool(truncated),
                                       'stalled': bool(info.get('stalled', False))}, allow_nan=False) + '\n')
                trace.flush()
                previous = pose
                if terminated or truncated:
                    # The frozen local-drive-v2 environment defines arrival as <= 0.35 m.
                    # Arrival wins if the goal is reached on the time-limit step.
                    outcome = 'success' if terminated and distance <= .35 else (
                        'terminal_failure' if terminated else 'timeout')
                    result.update(status='completed', outcome=outcome,
                                  terminated=bool(terminated), truncated=bool(truncated))
                    break
            else:
                result.update(status='completed', outcome='evaluation_limit', terminated=False, truncated=False)
    except Exception as exc:
        result.update(status='failed', outcome='infrastructure_error' if isinstance(exc, InfrastructureError)
                      else 'evaluation_error', error_type=type(exc).__name__, error=str(exc),
                      error_notes=getattr(exc, '__notes__', []))
    finally:
        try:
            env.close()
        except Exception as exc:
            result['cleanup_errors'].append(repr(exc))
        result['cleanup_errors'].extend(getattr(env.backend, 'cleanup_errors', []))
        if getattr(env.backend, 'name', None):
            result['cleanup_errors'].append('Backend container remains: ' + env.backend.name)
        if result['cleanup_errors'] and result['status'] == 'completed':
            result['observed_outcome'] = result['outcome']
            result.update(status='failed', outcome='cleanup_error')
        result['wall_seconds'] = time.monotonic() - started
    return result


def summarize(episodes, requested):
    completed = [e for e in episodes if e['status'] == 'completed']
    complete = len(completed) == requested
    counts = Counter(e['outcome'] for e in episodes)
    # Do not publish partial execution as an overall model quality result.
    return {
        'requested_episodes': requested, 'attempted_episodes': len(episodes),
        'completed_episodes': len(completed), 'outcome_counts': dict(counts),
        'success_rate': counts['success'] / requested if complete else None,
        'timeout_rate': counts['timeout'] / requested if complete else None,
        'mean_path_length_m': sum(e['path_length_m'] for e in completed) / requested if complete else None,
        'mean_episode_return': sum(e['episode_return'] for e in completed) / requested if complete else None,
        'mean_sim_elapsed_s': sum(e['sim_elapsed_s'] for e in completed) / requested if complete else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='New directory; existing results are never overwritten')
    parser.add_argument('--seeds', nargs='+', type=int, default=[7, 17, 23])
    parser.add_argument('--max-steps', type=int, default=600)
    parser.add_argument('--device', choices=['cpu', 'cuda', 'auto'], default='cpu')
    parser.add_argument('--code-revision', required=True, help='Repository revision before adding this evaluator; source hashes are recorded separately')
    args = parser.parse_args()
    if not 1 <= len(args.seeds) <= 20 or any(not 0 <= s < 2**31 for s in args.seeds):
        parser.error('Use 1..20 nonnegative int32 seeds')
    if not 1 <= args.max_steps <= 600:
        parser.error('max-steps must be 1..600 (120 simulation seconds)')
    checkpoint, manifest = checkpoint_for_evaluation(args.checkpoint)
    args.output.mkdir(parents=True, exist_ok=False)

    import torch
    from stable_baselines3 import SAC
    from .device import resolve_device
    from .envs.local_drive_env import LocalDriveEnv
    from .envs import local_drive_env, gazebo_backend

    torch.set_num_threads(1)
    device = resolve_device(args.device, torch.cuda.is_available())
    model = SAC.load(checkpoint / 'model.zip', device=device)
    before = {name: digest(checkpoint / name) for name in ('model.zip', 'replay.pkl', 'manifest.json')}
    report = {
        'status': 'running', 'started_at_utc': datetime.now(timezone.utc).isoformat(),
        'checkpoint': str(checkpoint), 'checkpoint_manifest': manifest, 'checkpoint_hashes_before': before,
        'code_base_revision': args.code_revision, 'deterministic': True, 'learning_enabled': False,
        'replay_loaded': False, 'device': device, 'scenario': 'indoor_fixed', 'seeds': args.seeds,
        'max_steps': args.max_steps, 'environment_time_limit_s': 120., 'arrival_threshold_m': .35,
        'source_sha256': {name: digest(path) for name, path in {
            'evaluate_vehicle.py': __file__, 'local_drive_env.py': local_drive_env.__file__,
            'gazebo_backend.py': gazebo_backend.__file__}.items()},
        'metric_definitions': {
            'path_length_m': 'Sum of XY truth-pose distances over returned 0.2s transitions; wheel-axis reference. Reset/cleanup motion excluded.',
            'episode_return': 'Undiscounted sum of returned rewards.',
            'timeout': 'Environment truncation without terminal arrival/failure. Evaluator step caps and infrastructure failures are separate.',
            'rates': 'All requested episodes; null if any episode is incomplete. Infrastructure errors stop the run.',
        },
        'limitations': ['Fixed map, initial state and forward 3m goal. Seeds are repeated baseline trials, not diverse scenarios.',
                        'No collision/contact ground-truth metric; no generalization or navigation quality claim from short training.'],
        'episodes': [],
    }
    stop = [False]
    def request_stop(*_):
        stop[0] = True
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    write_json(args.output / 'evaluation.json', report)
    try:
        for index, seed in enumerate(args.seeds):
            if stop[0]:
                break
            env = LocalDriveEnv()
            if index == 0:
                report['simulation_image'] = env.backend.image
                report['simulation_image_id'] = subprocess.check_output(
                    ['docker', 'image', 'inspect', env.backend.image, '--format', '{{.Id}}'], text=True, timeout=30).strip()
                report['simulation_rendering'] = 'GPU EGL' if env.backend.gpu else 'Mesa llvmpipe EGL'
            trace = args.output / f'episode-{index + 1:02d}.jsonl'
            episode = run_episode(model, env, seed, args.max_steps, trace, lambda: stop[0])
            episode['trace'] = trace.name
            report['episodes'].append(episode)
            report['metrics'] = summarize(report['episodes'], len(args.seeds))
            write_json(args.output / 'evaluation.json', report)
            print(json.dumps(episode), flush=True)
            if episode['status'] != 'completed':
                break
    finally:
        report['metrics'] = summarize(report['episodes'], len(args.seeds))
        report['checkpoint_hashes_after'] = {name: digest(checkpoint / name) for name in before}
        report['checkpoint_unchanged'] = before == report['checkpoint_hashes_after']
        completed = report['metrics']['completed_episodes'] == len(args.seeds)
        report['status'] = 'completed' if completed and report['checkpoint_unchanged'] else 'incomplete'
        report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        write_json(args.output / 'evaluation.json', report)
    print(json.dumps({'status': report['status'], **report['metrics']}, indent=2), flush=True)
    return 0 if report['status'] == 'completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
