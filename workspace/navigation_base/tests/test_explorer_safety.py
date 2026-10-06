"""Action-boundary regression tests; requires ROS messages, no running ROS graph."""
import math
import sys
import time
import unittest
import tempfile
import json
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'exploration'))
try:
    from explore import Explorer
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as NavPath
    import rclpy
    ROS_AVAILABLE = True
except ModuleNotFoundError as error:
    if error.name not in ('rclpy', 'nav2_msgs', 'geometry_msgs'):
        raise
    ROS_AVAILABLE = False


@unittest.skipUnless(ROS_AVAILABLE, 'run in the navigation image for ROS message types')
class ExplorerSafetyTests(unittest.TestCase):
    def setUp(self):
        # These fixtures exercise the retained radial baseline explicitly.
        # Footprint-mode geometry has its own contract/profile test suite.
        for target in ('explore.FOOTPRINT_MODE','safety_profile.FOOTPRINT_MODE'):
            active=patch(target,False);active.start();self.addCleanup(active.stop)

    def explorer(self):
        e = Explorer.__new__(Explorer)
        e.handle = None
        temporary=tempfile.TemporaryDirectory();self.addCleanup(temporary.cleanup)
        e.output=Path(temporary.name)/'trial.json'
        e.latest = {'nearest': 4., 'map_version': 'v1'}
        e.args = NS(wall_budget=100)
        e.started = time.monotonic()
        e.health = Mock()
        e.stop_confirmed = Mock(return_value=True)
        e.cancel_active = Mock(return_value=True)
        e.event = Mock()
        e.pose = Mock(return_value=[3., 5., 0.])
        e.wait = lambda future, seconds: future
        e.active_candidate = {'x': 7., 'y': 5., 'yaw': 0.}
        e.execution_path = [[3., 5., 0.], [7., 5., 0.]]
        e.path_revision = 0
        e.last_trace = e.last_progress = time.monotonic()
        e.spin_once = Mock()
        e.node = NS(get_clock=lambda: NS(now=lambda: rclpy.time.Time(seconds=10)))
        return e

    def client(self):
        pending = NS(done=lambda: False)
        return NS(send_goal_async=Mock(return_value=NS(accepted=True, get_result_async=lambda: pending)))

    def test_existing_guard_stop_never_dispatches_goal(self):
        e = self.explorer(); e.latest['nearest'] = 1.881
        client = self.client()
        result = e.execute(client, object(), 100, 'navigation')
        self.assertEqual(result['error_msg'], 'guard_stop')
        client.send_goal_async.assert_not_called()
        e.stop_confirmed.assert_called_once()

    def test_new_unknown_obstacle_cancels_immediately_without_no_progress_wait(self):
        e = self.explorer()
        e.spin_once.side_effect = lambda: e.latest.update(nearest=1.881)
        result = e.execute(self.client(), object(), 100, 'navigation')
        self.assertEqual(result['error_msg'], 'guard_stop')
        e.cancel_active.assert_called_once()

    def test_stop_must_be_confirmed(self):
        e = self.explorer(); e.latest['nearest'] = 1.881
        e.stop_confirmed.return_value = False
        client = self.client()
        with self.assertRaisesRegex(RuntimeError, 'stop_unconfirmed'):
            e.execute(client, object(), 100, 'navigation')
        client.send_goal_async.assert_not_called()

    def test_geometry_work_drains_pending_scan_without_extending_timeout(self):
        e = self.explorer()
        e.latest.update(scan_at=time.monotonic()-6., odom_at=time.monotonic(),
                        map_at=time.monotonic(), scan_valid=True)
        with patch('explore.rclpy.spin_once', side_effect=lambda *a, **kw: e.latest.update(scan_at=time.monotonic())):
            Explorer.health(e)
        e.latest['scan_at'] = time.monotonic()-6.
        with patch('explore.rclpy.spin_once'):
            with self.assertRaisesRegex(RuntimeError, 'scan_at_stale'):
                Explorer.health(e)

    def add_map(self, e, data):
        origin = NS(position=NS(x=0., y=0.), orientation=NS(w=1., x=0., y=0., z=0.))
        e.latest.update(data=data, map=NS(info=NS(resolution=.05, origin=origin),header=NS(stamp=NS(sec=10,nanosec=0))))
        t = NS(transform=NS(translation=NS(x=-.7057095, y=0.)))
        e.buffer = NS(lookup_transform=lambda *args: t)
        e.clearance_model = None

    def test_changed_map_cancels_before_guard_threshold(self):
        e = self.explorer(); data = np.zeros((200, 200), dtype=np.int16)
        self.add_map(e, data)
        self.assertTrue(e.evaluate_path(e.execution_path, e.active_candidate)['safe'])
        def update_map():
            changed = data.copy(); changed[135, 100] = 100
            e.latest.update(data=changed, map_version='v2')
        e.spin_once.side_effect = update_map
        result = e.execute(self.client(), object(), 100, 'navigation')
        self.assertEqual(result['error_msg'], 'path_guard_clearance')
        self.assertEqual(result['clearance']['map_version'], 'v2')
        self.assertGreater(e.latest['nearest'], 1.9)
        e.cancel_active.assert_called_once()

    def test_failure_snapshot_keeps_trigger_map_when_cancel_updates_map(self):
        e=self.explorer();self.add_map(e,np.zeros((200,200),dtype=np.int16))
        e.evaluate_path=Mock(return_value={'safe':False,'reason':'body_sweep_nonfree','map_version':'v1'})
        def canceled():
            e.latest.update(data=np.ones((200,200),dtype=np.int16)*100,map_version='v2')
            return True
        e.cancel_active.side_effect=canceled
        result=e.execute(self.client(),object(),100,'navigation')
        self.assertEqual(result['error_msg'],'body_sweep_nonfree')
        with np.load(e.output.with_suffix('.invalidated.npz')) as saved:
            self.assertTrue(np.all(saved['data']==0))
        saved=json.loads(e.output.with_suffix('.invalidated.json').read_text())
        self.assertEqual(saved['map_version'],'v1');self.assertEqual(saved['start'],[3.,5.,0.])
        self.assertTrue(saved['path']);e.cancel_active.assert_called_once()

    def test_live_replan_is_checked_and_old_plan_ignored(self):
        e = self.explorer(); data = np.zeros((200, 200), dtype=np.int16)
        data[160, 100] = 100
        self.add_map(e, data)
        self.assertTrue(e.evaluate_path(e.execution_path, e.active_candidate)['safe'])
        e.execution_started_ns = 10_000_000_000
        path = NavPath(); path.header.frame_id = 'map'; path.header.stamp.sec = 9
        for x, y in [(3., 5.), (5., 7.), (7., 5.)]:
            p = PoseStamped(); p.pose.position.x = x; p.pose.position.y = y; p.pose.orientation.w = 1.
            path.poses.append(p)
        e.path_cb(path)
        self.assertEqual(e.path_revision, 0)
        path.header.stamp.sec = 10
        e.spin_once.side_effect = lambda: e.path_cb(path)
        result = e.execute(self.client(), object(), 100, 'navigation')
        self.assertEqual(result['error_msg'], 'path_guard_clearance')
        self.assertEqual(e.path_revision, 1)
        e.cancel_active.assert_called_once()

    def test_missing_lidar_tf_fails_closed(self):
        e = self.explorer(); self.add_map(e, np.zeros((200, 200), dtype=np.int16))
        e.buffer.lookup_transform = Mock(side_effect=RuntimeError('missing'))
        with self.assertRaisesRegex(RuntimeError, 'lidar_transform_unavailable'):
            e.evaluate_path(e.execution_path, e.active_candidate)

    def test_nav2_successful_plan_can_be_rejected_before_navigation(self):
        e = self.explorer(); data = np.zeros((200, 200), dtype=np.int16)
        data[135, 100] = 100; self.add_map(e, data)
        path = NavPath(); path.header.frame_id = 'map'
        for x in [3., 7.]:
            p = PoseStamped(); p.pose.position.x = x; p.pose.position.y = 5.; p.pose.orientation.w = 1.
            path.poses.append(p)
        response = NS(status=4, result=NS(path=path))
        e.planner = NS(send_goal_async=lambda goal: NS(accepted=True, get_result_async=lambda: response))
        plan, outcome = e.plan(e.active_candidate)
        self.assertIsNone(plan)
        self.assertEqual(outcome['status'], 4)
        self.assertFalse(outcome['clearance']['safe'])

    def mission(self):
        e = self.explorer()
        e.args = NS(wall_budget=100, max_goals=2, initial_scan=False,
                    dry_run=False, policy='aggressive', goal_timeout=100)
        e.nav = e.planner = e.spin_client = NS(wait_for_server=lambda **kwargs: True)
        state = NS(wait_for_service=lambda **kwargs: True,
                   call_async=lambda req: NS(current_state=NS(id=3)))
        e.node.create_client = lambda *args: state
        e.node.destroy_client = e.node.destroy_node = Mock()
        e.buffer = NS(can_transform=lambda *args: True)
        e.observe = e.save_map = e.checkpoint = Mock()
        e.sample = lambda: {'known_area_m2': 1000.}
        e.latest.update(map=object(), scan_at=time.monotonic(), odom_at=time.monotonic())
        e.report = {}; e.history = []; e.goals = []; e.event_file = Mock(); e.lock = Mock(); e.motion_lock = Mock()
        candidates = [{'x': x, 'y': 5., 'yaw': 0., 'dx': x-3., 'dy': 0.,
                       'estimated_gain_m2': 10., 'euclidean_distance_m': x-3.} for x in (5., 8.)]
        def eligible(pose, excluded):
            return ([c for c in candidates if (c['x'], c['y']) not in excluded],
                    {'frontier_clusters': 1})
        e.model = lambda: NS(candidates=eligible, is_safe=lambda *args: True)
        e.plan = lambda c: ({'length': c['x']-3., 'path': [[c['x'], c['y'], c['yaw']]]}, {'status': 4})
        e.evaluate_path = lambda *args: {'safe': True}
        e.goal_pose = lambda c: PoseStamped()
        return e

    def test_canceled_goal_is_excluded_and_another_target_selected(self):
        e = self.mission()
        # Obstacle has cleared by the next decision. Even with large map gain,
        # the failed target must not be retried by aggressive revisit rules.
        e.execute = Mock(side_effect=[{'status': 5, 'error_msg': 'guard_stop'},
                                      {'status': 4, 'error_msg': ''}])
        self.assertTrue(e.run())
        self.assertEqual([g['candidate']['x'] for g in e.goals], [5., 8.])
        self.assertEqual([h['outcome'] for h in e.history], ['failed', 'visited'])
        self.assertEqual(e.execute.call_count, 2)

    def test_persistent_guard_stop_ends_mission_without_resubmission(self):
        e = self.mission()
        def blocked(*args):
            e.latest['nearest'] = 1.881
            return {'status': 5, 'error_msg': 'guard_stop'}
        e.execute = Mock(side_effect=blocked)
        self.assertTrue(e.run())
        self.assertEqual(e.report['stop_reason'], 'guard_blocked_no_safe_motion')
        self.assertTrue(e.report['stopped'])
        self.assertEqual(e.execute.call_count, 1)


if __name__ == '__main__':
    unittest.main()
