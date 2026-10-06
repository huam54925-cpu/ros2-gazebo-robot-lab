"""ROS-independent scan contract shared by planning and the final velocity guard."""
import math

STOP_RADIUS_M = 1.9
try:
    from .robot_contract import CONTRACT
except ImportError:
    from robot_contract import CONTRACT
SCAN_FRAME = CONTRACT['scan_frame']
BASE_FRAME = CONTRACT['base_frame']
SCAN_TIMEOUT_S = 1.5


def scan_state(scan):
    hits = [v for v in scan.ranges
            if math.isfinite(v) and scan.range_min <= v <= scan.range_max]
    valid = (scan.header.frame_id == SCAN_FRAME and bool(len(scan.ranges))
             and (bool(hits) or any(v == math.inf for v in scan.ranges)))
    return min(hits, default=math.inf), valid


def blocked_reason(command_age, scan_age, nearest, valid_scan):
    if command_age > CONTRACT['command_timeout_s']:
        return 'command timeout'
    if scan_age > SCAN_TIMEOUT_S or not valid_scan:
        return 'scan missing or stale'
    if nearest < STOP_RADIUS_M:
        return f'obstacle inside {STOP_RADIUS_M:g} m stop radius'
    return ''
