import json
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'exploration'))
from path_safety import PathSafety, remaining_path
from safety_contract import STOP_RADIUS_M, blocked_reason, scan_state


class PathSafetyTests(unittest.TestCase):
    def model(self, obstacle=None, origin=(0, 0, 0), version='v1'):
        data = np.zeros((200, 200), dtype=np.int16)
        if obstacle is not None:
            x, y = obstacle
            data[int(y/.05), int(x/.05)] = 100
        return PathSafety(data, .05, origin, version)

    def test_explicit_reduced_path_clearance_does_not_change_scan_guard(self):
        model=self.model((5,5))
        start,goal=[2,3.49,0],[8,3.49,0]
        self.assertFalse(model.evaluate([start,goal],start,goal,(0,0))['safe'])
        result=model.evaluate([start,goal],start,goal,(0,0),1.48)
        self.assertTrue(result['safe'])
        self.assertEqual(result['guard_threshold_m'],1.9)
        self.assertLess(result['guard_margin_m'],0)
        self.assertIsNone(result['planning_margin_m'])
        start,goal=[2,3.50,0],[8,3.50,0]
        self.assertFalse(model.evaluate([start,goal],start,goal,(0,0),1.48)['safe'])
        with self.assertRaises(ValueError):model.evaluate([start,goal],start,goal,(0,0),1.0)

    def test_safe_endpoints_do_not_hide_unsafe_middle(self):
        model = self.model((5, 5))
        start, goal = [2, 3.1, 0], [8, 3.1, 0]
        for pose in (start, goal):
            self.assertTrue(model.evaluate([pose], pose, pose, (0, 0))['safe'])
        result = model.evaluate([start, goal], start, goal, (0, 0))
        self.assertFalse(result['safe'])
        self.assertLess(result['predicted_min_clearance_m'], STOP_RADIUS_M)
        # A genuinely different route remains selectable.
        detour = [start, [2, 1.5, 0], [8, 1.5, 0], goal]
        self.assertTrue(model.evaluate(detour, start, goal, (0, 0))['safe'])

    def test_offset_turn_sweep_blocks_despite_safe_start_and_finish(self):
        model = self.model((5, 2.7))
        start, goal = [5, 5, 0], [5, 5, math.pi]
        for pose in (start, goal):
            self.assertTrue(model.evaluate([pose], pose, pose, (-.7057095, 0))['safe'])
        result = model.evaluate([start, goal], start, goal, (-.7057095, 0))
        self.assertFalse(result['safe'])
        self.assertLess(result['predicted_min_clearance_m'], 1.9)
        self.assertTrue(model.evaluate([start, goal], start, goal, (0, 0))['safe'])

    def test_translation_tangent_and_final_rotation_are_checked(self):
        model = self.model((5, 2.7))
        # Identity quaternions along a northbound path must not hide the lidar
        # swinging south when the controller aligns to the path tangent.
        start, goal = [5, 5, 0], [5, 8, 0]
        self.assertFalse(model.evaluate([start, goal], start, goal, (-.7057095, 0))['safe'])
        # Goal yaw differs from the last planner pose.
        start = [5, 5, 0]
        self.assertFalse(model.evaluate([start], start, [5, 5, math.pi], (-.7057095, 0))['safe'])

    def test_sparse_segment_sweeps_translation_and_rotation_together(self):
        model = self.model((5, 2.7))
        start, goal = [2, 5, 0], [8, 5, math.pi]
        # The straight tangent trajectory and in-place endpoint turns are safe.
        self.assertTrue(model.evaluate([start, [8, 5, 0]], start, goal, (-.7057095, 0))['safe'])
        # Interpolating yaw from 0 to pi while moving brings the lidar south
        # near the middle; endpoints and centreline clearance would miss it.
        self.assertFalse(model.evaluate([start, goal], start, goal, (-.7057095, 0))['safe'])

    def test_map_change_invalidates_same_path(self):
        start, goal = [2, 3.1, 0], [8, 3.1, 0]
        self.assertTrue(self.model().evaluate([start, goal], start, goal, (0, 0))['safe'])
        changed = self.model((5, 5), version='v2').evaluate([start, goal], start, goal, (0, 0))
        self.assertFalse(changed['safe'])
        self.assertEqual(changed['map_version'], 'v2')

    def test_rotated_map_origin_preserves_distances(self):
        start, goal = [2, 3.1, 0], [8, 3.1, 0]
        a = .71
        def transform(p):
            return [20+math.cos(a)*p[0]-math.sin(a)*p[1],
                    -3+math.sin(a)*p[0]+math.cos(a)*p[1], p[2]+a]
        expected = self.model((5, 5)).evaluate([start, goal], start, goal, (-.7, .2))
        got = self.model((5, 5), (20, -3, a)).evaluate(
            [transform(start), transform(goal)], transform(start), transform(goal), (-.7, .2))
        self.assertAlmostEqual(expected['predicted_min_clearance_m'], got['predicted_min_clearance_m'])

    def test_unknown_still_requires_runtime_guard(self):
        data = np.full((100, 100), -1)
        pose = [2, 2, 0]
        check = PathSafety(data, .1, (0, 0, 0), 'unknown').evaluate([pose], pose, pose, (0, 0))
        self.assertTrue(check['safe'])
        self.assertTrue(check['known_obstacles_only'])
        self.assertIn('obstacle', blocked_reason(0, 0, 1.881, True))
        self.assertEqual(blocked_reason(0, 0, 1.9, True), '')
        self.assertIn('stale', blocked_reason(0, 1.51, 9, True))
        self.assertIn('timeout', blocked_reason(.51, 0, 9, True))

    def test_scan_contract_invalid_and_infinite_returns(self):
        scan = SimpleNamespace(header=SimpleNamespace(frame_id='vehicle/lidar'),
                               range_min=.1, range_max=12, ranges=[math.nan, math.inf])
        self.assertEqual(scan_state(scan), (math.inf, True))
        scan.ranges = [math.nan]
        self.assertFalse(scan_state(scan)[1])
        scan.ranges = [1.881, 9]
        self.assertEqual(scan_state(scan), (1.881, True))
        scan.header.frame_id = 'wrong'
        self.assertFalse(scan_state(scan)[1])

    def test_invalid_paths_fail_closed(self):
        model = self.model()
        pose = [5, 5, 0]
        for path in ([], [[math.nan, 5, 0]], [[1, 1, 0]], [[5, 5]]):
            self.assertFalse(model.evaluate(path, pose, pose, (0, 0))['safe'])
        self.assertFalse(model.evaluate([pose], pose, pose, (20, 0))['safe'])

    def test_crossing_keeps_earliest_possible_remaining_route(self):
        path = [[1, 1, 0], [5, 5, 0], [8, 8, 0], [5, 5, 0], [9, 1, 0]]
        self.assertEqual(remaining_path(path, [5, 5, 0]), path[1:])

    def test_remaining_path_does_not_turn_back_to_traversed_waypoint(self):
        path = [[2, 5, 0], [4, 5, 0], [6, 5, 0]]
        self.assertEqual(remaining_path(path, [4.5, 5, 0]), path[2:])

    def test_historical_eighth_goal_endpoint_passes_route_fails(self):
        root = Path(__file__).resolve().parents[3]/'docs/experiments/2026-10-03/frontier-comparison'
        report = json.loads((root/'frontier-aggressive-20261003.json').read_text())
        goal = report['goals'][7]
        snapshot = np.load(root/'frontier-aggressive-20261003.final.npz')
        model = PathSafety(snapshot['data'], float(snapshot['resolution']), snapshot['origin'], 'historical-final')
        c = goal['candidate']; end = [c['x'], c['y'], c['yaw']]
        self.assertTrue(model.evaluate([end], end, end, (-.7057095, 0))['safe'])
        # Historical logs only recorded XY; infer forward tangents for replay.
        path = goal['plan']
        poses = [[*p, math.atan2(path[i+1][1]-p[1], path[i+1][0]-p[0])
                  if i+1 < len(path) else end[2]] for i, p in enumerate(path)]
        result = model.evaluate(poses, goal['before']['pose'], end, (-.7057095, 0))
        self.assertFalse(result['safe'])
        self.assertLess(result['predicted_min_clearance_m'], 1.9)


if __name__ == '__main__':
    unittest.main()
