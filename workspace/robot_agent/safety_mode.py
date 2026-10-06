"""Resolve the optional collision mount once per run, never per AI action."""
import os


def resolve(mode='auto', environment='gazebo'):
    if mode not in ('off', 'on', 'auto'):
        raise ValueError('safety_mode_must_be_off_on_or_auto')
    enabled = mode == 'on' or (mode == 'auto' and environment != 'gazebo')
    return {'requested': mode, 'enabled': enabled,
            'resolved': 'on' if enabled else 'off', 'environment': environment,
            'reason': ('explicit_' + mode if mode != 'auto' else
                       'gazebo_experiment_defaults_off' if environment == 'gazebo' else
                       'non_gazebo_defaults_on'),
            'mount': 'lidar_sweep' if enabled else None}


def runtime_policy(environ=None):
    env = os.environ if environ is None else environ
    if env.get('ROBOT_RUNTIME') != 'scan-drive-v2':
        raise RuntimeError('v2_gazebo_launch_required')
    if env.get('ROBOT_ENVIRONMENT') != 'gazebo':
        raise RuntimeError('only_gazebo_runtime_implemented')
    return resolve(env.get('ROBOT_SAFETY_MOUNT', 'auto'), env['ROBOT_ENVIRONMENT'])


def load_mount(policy):
    if not policy['enabled']:
        return None
    from mounts.lidar_sweep import LidarSweep
    return LidarSweep()
