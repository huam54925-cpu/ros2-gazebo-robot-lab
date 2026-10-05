"""Explainable regional decision context. No safety authority or motion API."""
from navigation_base.exploration.regions import region_id, SECTOR_M


def context(catalog, tasks, mission_id=None):
    pose = catalog.get('robot_pose')
    current = region_id(pose) if pose else None
    epoch = catalog.get('region_epoch')
    owned = [t for t in tasks if epoch and (t.get('candidate') or {}).get('region_epoch') == epoch]
    owned.sort(key=lambda t: t.get('created_unix_s', 0))
    regions = {}
    for c in catalog.get('candidates', []):
        rid = c.get('target_region_id', c.get('region_id'))
        if not rid:
            continue
        entry = regions.setdefault(rid, {'region_id': rid, 'safe_candidate_ids': [],
            'potential_proxy_m2': 0, 'successful_arrivals': 0, 'failed_attempts': 0})
        entry['safe_candidate_ids'].append(c['frontier_id'])
        entry['potential_proxy_m2'] = max(entry['potential_proxy_m2'], c.get('regional_potential_proxy_m2', 0),
                                          c.get('novel_unknown_area_proxy_m2', 0))
    arrivals = {}
    evidence = []
    for task in owned:
        result = task.get('result') or {}
        after = result.get('after', {}).get('pose')
        rid = region_id(after) if after else task['candidate'].get('region_id')
        if task['status'] == 'succeeded' and after and task['candidate'].get('type') != 'rotate':
            arrivals[rid] = arrivals.get(rid, 0) + 1
        target = task['candidate'].get('target_region_id', rid)
        if target in regions and task['status'] in ('aborted', 'rejected', 'canceled'):
            regions[target]['failed_attempts'] += 1
        before = result.get('before', {}).get('pose')
        if rid == current and before and region_id(before) == current and task['status'] == 'succeeded':
            gain = result.get('map_gain', {})
            evidence.append({'task_id': task['task_id'], 'usable': gain.get('usable_for_trend', False),
                'area_m2': gain.get('observed_new_known_area_m2'),
                'gain_status': gain.get('gain_status', 'NOT_MEASURED')})
    window = evidence[-3:]
    # A conservative, explicitly provisional threshold; invalid windows never
    # count as zero. This describes map-window yield, not causal information gain.
    low = len(window) == 3 and all(e['usable'] and e['area_m2'] is not None and e['area_m2'] < .5 for e in window)
    for rid, entry in regions.items():
        entry['successful_arrivals'] = arrivals.get(rid, 0)
    alternatives = [r for r in regions if r != current]
    local = regions.get(current, {}).get('potential_proxy_m2', 0)
    saturated = low and local < .5 and catalog.get('search_complete', False)
    rationale = ('local_window_yield_and_remaining_proxy_low' if saturated else
                 'diversify_after_repeated_local_arrivals' if arrivals.get(current, 0) >= 2 and alternatives else
                 'no_local_safe_candidate' if current not in regions and alternatives else 'compare_region_return_and_cost')
    expand = bool(alternatives and (saturated or arrivals.get(current, 0) >= 2 or current not in regions))
    commitment = None
    recent = [t for t in owned if t.get('mission_id') == mission_id] if mission_id else []
    if recent:
        last = recent[-1]
        target = last['candidate'].get('target_region_id')
        streak = 0
        for task in reversed(recent):
            if task['status'] != 'succeeded' or task['candidate'].get('target_region_id') != target:
                break
            streak += 1
        if last['status'] == 'succeeded' and last['candidate'].get('transit') and target in regions and streak < 3:
            commitment = target
    return {'region_epoch': epoch, 'sector_size_m': SECTOR_M, 'current_region_id': current,
        'regions': list(regions.values()), 'arrivals_by_region': arrivals,
        'recent_local_gain_windows': window,
        'local_gain_evidence': 'LOW_WINDOW_YIELD' if low else 'INSUFFICIENT' if len(window) < 3 or not all(e['usable'] for e in window) else 'AVAILABLE',
        'local_saturated': saturated, 'recommended_mode': 'EXPAND_REGION' if expand or commitment else 'LOCAL_EXPLORE',
        'recommendation_reason': rationale, 'committed_region_id': commitment,
        'search_complete': catalog.get('search_complete', False),
        'unknown_space':catalog.get('unknown_space',{'completion_evidence':'NOT_ANALYZED'}),
        'coverage_complete': False, 'scope': 'current_slam_map_spatial_sectors_not_rooms'}


def rank(available, regional):
    """Deterministic baseline; LLM receives the same scores and recommendations."""
    preferred = regional.get('committed_region_id')
    current = regional.get('current_region_id')
    arrivals = regional.get('arrivals_by_region', {})
    def score(key):
        action, c = available[key]
        target = c.get('target_region_id', c.get('region_id'))
        priority = 2 if preferred and target == preferred else 1 if (
            regional.get('recommended_mode') == 'EXPAND_REGION' and action == 'MOVE' and target != current) else 0
        value = c.get('regional_score', c.get('joint_score', c.get('score', 0))) / (1 + .5*arrivals.get(target, 0))
        return priority, value
    return max(available, key=score)
