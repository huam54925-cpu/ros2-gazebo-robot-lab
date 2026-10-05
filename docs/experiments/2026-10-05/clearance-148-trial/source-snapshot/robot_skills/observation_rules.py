"""Persistent spatial repetition rules, independent of changing catalog IDs."""
import math


def exclusion(option, tasks):
    position=option['start_pose'] if option['type']=='rotate' else [option['x'],option['y']]
    nearby=[]
    for task in tasks:
        old=task.get('candidate') or {}
        if task['kind']!='perform_observation' or not task['accepted'] or not old: continue
        point=old.get('start_pose',[]) if old.get('type')=='rotate' else [old.get('x',1e9),old.get('y',1e9)]
        if len(point)>=2 and math.dist(position[:2],point[:2])<1.: nearby.append(task)
    if option['type']=='move_observe':
        return 'observation_position_already_attempted' if nearby else None
    rotations=[t for t in nearby if t['candidate'].get('type')=='rotate']
    angle=option['signed_angle_rad']
    if any(abs(t['candidate']['signed_angle_rad']-angle)<1e-6 for t in rotations):
        return 'rotation_already_attempted_nearby'
    if sum(abs(t['candidate']['signed_angle_rad']) for t in rotations)+abs(angle)>math.pi+1e-6:
        return 'local_rotation_budget'
    if sum((t.get('result') or {}).get('observation_metrics',{}).get('gain_status')=='LOW_GAIN' for t in rotations)>=2:
        return 'repeated_low_gain'
    return None
