"""Read-only shared vehicle geometry; JSON is a dependency-free YAML subset.

No runtime activation, ROS startup or parameter mutation occurs on import.
"""
import hashlib
import json
import math
from pathlib import Path

PATH = Path(__file__).resolve().parent / 'config/robot_contract.yaml'


def load(path=PATH):
    raw = Path(path).read_bytes()
    value = json.loads(raw)
    points = value['footprint']
    if len(points) < 3 or any(len(p) != 2 or any(not math.isfinite(x) for x in p) for p in points):
        raise ValueError('invalid_contract_footprint')
    cross = [(points[(i+1) % len(points)][0]-p[0]) *
             (points[(i+2) % len(points)][1]-points[(i+1) % len(points)][1]) -
             (points[(i+1) % len(points)][1]-p[1]) *
             (points[(i+2) % len(points)][0]-points[(i+1) % len(points)][0])
             for i, p in enumerate(points)]
    if not (all(c > 0 for c in cross) or all(c < 0 for c in cross)):
        raise ValueError('contract_requires_convex_footprint')
    for key in ('body_padding_m', 'motion_sample_step_m', 'linear_deceleration_m_s2',
                'angular_deceleration_rad_s2', 'reaction_time_s', 'guard_sample_step_m',
                'laser_hit_square_m', 'command_timeout_s', 'feedback_wall_timeout_s', 'command_wall_timeout_s', 'scan_pose_gap_s'):
        if isinstance(value[key], bool) or not math.isfinite(value[key]) or value[key] <= 0:
            raise ValueError('invalid_contract_'+key)
    for profile in value['profiles'].values():
        if any(not math.isfinite(x) or x <= 0 for x in profile.values()):
            raise ValueError('invalid_contract_profile')
    short = value['short_motion']
    for key, number in short.items():
        numbers = number if key == 'probe_steps_m' else [number]
        if not numbers or any(isinstance(x, bool) or not isinstance(x, (int, float))
                              or not math.isfinite(x) or x <= 0 for x in numbers):
            raise ValueError('invalid_short_motion_'+key)
    if (short['speed_m_s'] > value['profiles']['footprint_075']['max_linear_m_s']
            or short['minimum_step_m'] > short['reverse_max_m']
            or short['probe_steps_m'] != sorted(short['probe_steps_m'], reverse=True)
            or min(short['probe_steps_m']) < short['minimum_step_m']
            or any(int(short[k]) != short[k] for k in ('max_recoveries','max_recoveries_per_location'))):
        raise ValueError('invalid_short_motion_limits')
    value['sha256'] = hashlib.sha256(raw).hexdigest()
    return value


CONTRACT = load()
CONTRACT_HASH = CONTRACT['sha256']


def nav2_parameters(config, profile):
    """Derive both costmaps and motion bounds from the same contract."""
    cfg = json.loads(json.dumps(config))
    limits = CONTRACT['profiles'][profile]
    for name in ('local_costmap', 'global_costmap'):
        params = cfg[name][name]['ros__parameters']
        params.update(footprint=json.dumps(CONTRACT['footprint']),
                      footprint_padding=CONTRACT['body_padding_m'],
                      robot_base_frame=CONTRACT['base_frame'], track_unknown_space=True)
    follow = cfg['controller_server']['ros__parameters']['FollowPath']
    follow.update(max_linear_vel=min(follow['max_linear_vel'], limits['max_linear_m_s']),
                  max_angular_vel=min(follow['max_angular_vel'], limits['max_angular_rad_s']),
                  min_angular_vel=-min(abs(follow['min_angular_vel']), limits['max_angular_rad_s']),
                  max_linear_decel=-CONTRACT['linear_deceleration_m_s2'],
                  max_angular_decel=-CONTRACT['angular_deceleration_rad_s2'])
    follow['rotate_to_heading_angular_vel']=min(follow['rotate_to_heading_angular_vel'],limits['max_angular_rad_s'])
    follow['regulated_linear_scaling_min_speed']=min(follow['regulated_linear_scaling_min_speed'],limits['max_linear_m_s'])
    follow['min_approach_linear_velocity']=min(follow['min_approach_linear_velocity'],limits['max_linear_m_s'])
    behavior = cfg['behavior_server']['ros__parameters']
    behavior['robot_base_frame'] = CONTRACT['base_frame']
    behavior['local_frame'] = CONTRACT['odom_frame']
    behavior['max_rotational_vel'] = limits['max_angular_rad_s']
    behavior['min_rotational_vel'] = min(behavior['min_rotational_vel'],limits['max_angular_rad_s'])
    behavior['rotational_acc_lim'] = CONTRACT['angular_deceleration_rad_s2']
    for name in ('backup', 'drive_on_heading'):
        behavior[name+'.acceleration_limit'] = CONTRACT['linear_deceleration_m_s2']
        behavior[name+'.deceleration_limit'] = -CONTRACT['linear_deceleration_m_s2']
        behavior[name+'.minimum_speed'] = CONTRACT['short_motion']['speed_m_s']
    return cfg
