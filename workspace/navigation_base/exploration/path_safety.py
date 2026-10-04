"""Map-only, sensor-centred clearance checks for complete Nav2 candidate paths.

Unknown cells are not invented obstacles. A passing result only covers known
occupied cells; the scan watchdog and independent guard remain authoritative.
"""
import math
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from safety_contract import STOP_RADIUS_M

PLANNING_MARGIN_M = 0.25  # tracking / localization / map uncertainty allowance
SAMPLE_STEP_M = 0.05


def angle_delta(a, b):
    return math.atan2(math.sin(b-a), math.cos(b-a))


def remaining_path(path, pose):
    """Discard traversed points; evaluate from the actual current pose onward.

At crossings choose the earliest equally near point, conservatively keeping
loops rather than skipping an untraversed part of the route.
"""
    if len(path) < 2:
        return path
    point = np.asarray(pose[:2])
    distances = []
    for a, b in zip(path, path[1:]):
        a, b = np.asarray(a[:2]), np.asarray(b[:2])
        delta = b-a
        t = np.clip(np.dot(point-a, delta)/max(float(np.dot(delta, delta)), 1e-12), 0, 1)
        distances.append(float(np.linalg.norm(point-(a+t*delta))))
    nearest = min(distances)
    index = next(i for i, d in enumerate(distances) if d <= nearest + 0.01)
    return path[index+1:]


class PathSafety:
    def __init__(self, data, resolution, origin, map_version):
        self.data = np.asarray(data)
        self.resolution = float(resolution)
        self.origin = tuple(origin)
        self.map_version = map_version
        if (self.data.ndim != 2 or not math.isfinite(self.resolution)
                or self.resolution <= 0 or len(origin) != 3
                or not all(math.isfinite(v) for v in origin)):
            raise ValueError('invalid_clearance_map')
        # Non-free evidence includes uncertain occupancy; unknown (-1) remains
        # unknown. Distances are to occupied cell SQUARES, not just their centres.
        cells = np.argwhere(self.data >= 25)
        self.obstacles = (cells[:, ::-1] + 0.5) * self.resolution

    def _local(self, points):
        ox, oy, a = self.origin
        c, s = math.cos(a), math.sin(a)
        return (points - [ox, oy]) @ np.array([[c, -s], [s, c]])

    def evaluate(self, path, start, goal, sensor_offset):
        result = {'safe': False, 'map_version': self.map_version,
                  'guard_threshold_m': STOP_RADIUS_M,
                  'planning_margin_m': PLANNING_MARGIN_M,
                  'required_clearance_m': STOP_RADIUS_M + PLANNING_MARGIN_M,
                  'sensor_offset_m': list(sensor_offset),
                  'predicted_min_clearance_m': None, 'guard_margin_m': None,
                  'known_obstacles_only': True}
        if (not path or any(len(p) != 3 for p in [*path, start, goal])
                or len(sensor_offset) != 2
                or not all(math.isfinite(v) for p in [*path, start, goal, sensor_offset] for v in p)):
            return {**result, 'reason': 'invalid_path_or_pose'}
        if math.dist(path[-1][:2], goal[:2]) > 0.25:
            return {**result, 'reason': 'path_goal_mismatch'}
        # Include actual start orientation, stored planner orientations, forward
        # segment tangents and the final goal rotation. Smac2D quaternion yaw
        # alone is not the controller's heading on every part of a curved path.
        nodes = [list(start)]
        for p in [*path, goal]:
            previous = nodes[-1]
            if math.dist(previous[:2], p[:2]) > 1e-8:
                tangent = math.atan2(p[1]-previous[1], p[0]-previous[0])
                nodes.extend([[*previous[:2], tangent], [*p[:2], tangent]])
            nodes.append(list(p))
        samples = [nodes[0]]
        offset_radius = math.hypot(*sensor_offset)
        # Also sweep interpolated SE(2) planner poses: a sparse segment can
        # translate and rotate simultaneously, unlike tangent-following motion.
        for chain in (nodes, [start, *path, goal]):
            for a, b in zip(chain, chain[1:]):
                turn = angle_delta(a[2], b[2])
                # Upper bound even during offset-lidar translation + rotation.
                travel = math.dist(a[:2], b[:2]) + offset_radius * abs(turn)
                count = max(1, math.ceil(travel / SAMPLE_STEP_M))
                for fraction in np.arange(1, count+1) / count:
                    samples.append([a[0]+fraction*(b[0]-a[0]),
                                    a[1]+fraction*(b[1]-a[1]), a[2]+fraction*turn])
        samples = np.asarray(samples)
        c, s = np.cos(samples[:, 2]), np.sin(samples[:, 2])
        sx, sy = sensor_offset
        sensors = samples[:, :2] + np.column_stack((c*sx-s*sy, s*sx+c*sy))
        local = self._local(sensors)
        cells = np.floor(local/self.resolution).astype(int)
        h, w = self.data.shape
        if np.any((cells[:, 0] < 0) | (cells[:, 0] >= w)
                  | (cells[:, 1] < 0) | (cells[:, 1] >= h)):
            return {**result, 'reason': 'sensor_outside_map'}
        minimum = math.inf
        worst = 0
        # Bounded intermediate memory; no scipy / new system dependency.
        for first in range(0, len(local), 128):
            clearance = np.full(min(128, len(local)-first), np.inf)
            for offset in range(0, len(self.obstacles), 512):
                delta = np.maximum(np.abs(local[first:first+128, None, :]
                                   - self.obstacles[None, offset:offset+512, :])
                                   - self.resolution/2, 0)
                clearance = np.minimum(clearance, np.sqrt((delta*delta).sum(axis=2)).min(axis=1))
            index = int(np.argmin(clearance))
            if clearance[index] < minimum:
                minimum = float(clearance[index]); worst = first+index
        # Distance to a closed obstacle set is 1-Lipschitz. Subtract half the
        # maximum sensor travel between samples to cover the continuous sweep.
        lower_bound = max(0., minimum-SAMPLE_STEP_M/2)
        safe = lower_bound >= result['required_clearance_m']
        return {**result, 'safe': safe,
                'reason': 'clear' if safe else 'path_guard_clearance',
                'predicted_min_clearance_m': lower_bound if math.isfinite(lower_bound) else None,
                'guard_margin_m': lower_bound-STOP_RADIUS_M if math.isfinite(lower_bound) else None,
                'worst_base_pose': samples[worst].tolist(),
                'worst_sensor_xy': sensors[worst].tolist(), 'samples': len(samples)}
