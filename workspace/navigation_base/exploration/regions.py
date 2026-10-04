"""Map-frame spatial sectors and bounded transit; no ground truth or commands.

Sectors are bookkeeping cells, not detected rooms or a claim of map coverage.
An epoch is supplied by the local catalog owner to scope persistent IDs.
"""
import math

SECTOR_M = 4.0


def region_id(point):
    return 'R_%d_%d' % (math.floor(point[0] / SECTOR_M), math.floor(point[1] / SECTOR_M))


def tier(distance):
    return 'local' if distance <= 3 else 'regional' if distance <= 6 else 'distant'


def diverse_candidates(candidates, limit):
    """One per sector before second choices, interleaved by distance tier.

    Euclidean tiers only schedule planning; path length determines final tiers.
    Retain a near alternative without allowing it to consume the whole shortlist.
    """
    buckets = {}
    for item in sorted(candidates, key=lambda c: c['preplan_score'], reverse=True):
        item = dict(item, region_id=region_id([item['x'], item['y']]))
        buckets.setdefault(item['region_id'], []).append(item)
    order = []
    tiers = {name: [] for name in ('local', 'regional', 'distant')}
    for key, items in buckets.items():
        tiers[tier(items[0]['euclidean_distance_m'])].append(key)
    while any(tiers.values()):
        for values in tiers.values():
            if values:
                order.append(values.pop(0))
    selected = []
    while len(selected) < limit and any(buckets.values()):
        for key in order:
            if buckets[key] and len(selected) < limit:
                selected.append(buckets[key].pop(0))
    return selected


def transit_exclusions(tasks, candidate):
    """Failed poses stay excluded; a successful corridor may be reused twice.

    Reuse is bounded per destination sector and map epoch, across missions, so
    restarting a session cannot keep repeating the same unproductive relay.
    """
    accepted = [t for t in tasks if t.get('accepted') and t.get('candidate')]
    blocked = [[t['candidate']['x'], t['candidate']['y']] for t in accepted
               if t['status'] != 'succeeded']
    relays = [t['candidate'] for t in accepted if t['status'] == 'succeeded'
              and t['candidate'].get('transit')
              and t['candidate'].get('region_epoch') == candidate.get('region_epoch')
              and t['candidate'].get('target_region_id') == candidate['region_id']]
    for relay in relays:
        point = [relay['x'], relay['y']]
        if sum(math.dist(point, [r['x'], r['y']]) < 1 for r in relays) >= 2:
            blocked.append(point)
    return blocked


def transit_candidates(candidate, path, start, excluded=(), segment_m=5.5):
    """Try up to six spaced existing path points, farthest first.

    Caller must replan and check the resulting pose/path before publishing it.
    """
    walked = 0.0
    previous = start
    options = []
    for point in path:
        distance = math.dist(previous[:2], point[:2])
        walked += distance
        if walked > segment_m:
            break
        if distance > 1e-6:
            heading = math.atan2(point[1]-previous[1], point[0]-previous[0])
            if walked >= 2 and not any(math.dist(point[:2], p[:2]) < 1 for p in excluded):
                options.append((point, heading))
        previous = point
    selected = []
    for point, heading in reversed(options):
        if any(math.dist(point[:2], [r['x'], r['y']]) < .5 for r in selected):
            continue
        selected.append({**candidate, 'x': point[0], 'y': point[1], 'yaw': heading,
            'dx': point[0]-start[0], 'dy': point[1]-start[1],
            'euclidean_distance_m': math.dist(point[:2], start[:2]),
            'regional_target_frontier_distance_m': candidate.get('frontier_distance_m'),
            'frontier_distance_m': None,
            'target_region_id': candidate['region_id'],
            'region_id': region_id(point), 'transit': True,
            'regional_target': [candidate['x'], candidate['y'], candidate['yaw']]})
        if len(selected) == 6:
            break
    return selected


def transit_candidate(candidate, path, start, excluded=(), segment_m=5.5):
    options = transit_candidates(candidate, path, start, excluded, segment_m)
    return options[0] if options else None
