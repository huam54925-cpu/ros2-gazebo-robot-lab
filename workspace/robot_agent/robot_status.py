"""Read-only, freshness-aware access to atomic ROS snapshots. No ROS publishers."""
import json
import math
from pathlib import Path
import time

SNAPSHOT = Path(__file__).resolve().parents[1] / 'log' / 'robot-status.json'
LIMITS = {'map': 30.0, 'navigation': 3.0, 'requested_velocity': 0.5}


def get_robot_status(path=SNAPSHOT, now=None):
    now = time.monotonic() if now is None else now
    try:
        data = json.loads(path.read_text())
        age = now - data['generated_monotonic_s']
        if not math.isfinite(age) or age < 0 or age > 3 or data['schema_version'] != 1:
            return {'status': 'unavailable', 'reason': 'snapshot_stale_or_invalid', 'mode': 'read_only'}
        clock = data['sources'].get('clock', {})
        sim_now = clock.get('sim_time_s')
        for name, source in data['sources'].items():
            received = now - source.pop('received_monotonic_s')
            limit = LIMITS.get(name, 1.5)
            source['received_age_wall_s'] = round(received, 3)
            source['fresh'] = math.isfinite(received) and 0 <= received <= limit
            if 'stamp_sim_s' in source and sim_now is not None:
                sim_age = sim_now - source['stamp_sim_s']
                source['stamp_age_sim_s'] = round(sim_age, 3)
                source['fresh'] &= -0.1 <= sim_age <= limit
        data['snapshot_age_wall_s'] = round(age, 3)
        data['status'] = 'available'
        data['motion_tools_enabled'] = False
        data['note'] = ('Read-only observations, not permission to move. Stale/missing fields are unknown. '
                        'Guard reason is inferred, not a guard acknowledgement. No navigation status '
                        'message means unknown, not proof of idle. Map known area is not room coverage.')
        return data
    except (OSError, ValueError, TypeError, KeyError):
        return {'status': 'unavailable', 'reason': 'snapshot_missing_or_invalid', 'mode': 'read_only'}
