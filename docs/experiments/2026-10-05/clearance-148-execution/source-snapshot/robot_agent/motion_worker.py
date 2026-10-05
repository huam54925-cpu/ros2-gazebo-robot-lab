"""One simulation-only 0.4 m Nav2 trial, with local safety and cancellation."""
import argparse
import fcntl
import json
import math
from pathlib import Path
import signal
import sys
import time
from types import SimpleNamespace
import uuid

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'navigation_base' / 'exploration'))
sys.path.insert(0, str(HERE.parent / 'navigation_base'))
from explore import Explorer, GetState, NavigateToPose, ManageLifecycleNodes, rclpy
from rclpy.signals import SignalHandlerOptions

DIRECTORY = HERE.parent / 'log' / 'motion-trials'


class Trial(Explorer):
    def __init__(self, identifier):
        self.identifier = identifier
        self.cancel_path = DIRECTORY / (identifier + '.cancel')
        self.dispatched = False
        self.pending_goal = None
        self.start_pose = None
        self.canceling = False
        super().__init__(SimpleNamespace(output=str(DIRECTORY / (identifier + '.detail.json')),
                                         policy='aggressive', wall_budget=100, max_goals=1,
                                         goal_timeout=45))

    def health(self):
        super().health()
        if not self.canceling and self.cancel_path.exists():
            raise RuntimeError('operator_canceled')
        if self.distance > .65:
            raise RuntimeError('trial_distance_budget')

    def execute(self, client, goal, timeout, kind):
        # Record the send future so a late acceptance is canceled after timeout.
        original = client.send_goal_async
        def send(message):
            self.dispatched = True
            self.pending_goal = original(message)
            return self.pending_goal
        client.send_goal_async = send
        try:
            return super().execute(client, goal, timeout, kind)
        finally:
            client.send_goal_async = original

    def pause_navigation(self):
        if not self.manager.wait_for_service(timeout_sec=2):
            return False
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.PAUSE
        try:
            return bool(self.wait(self.manager.call_async(request), 8).success)
        except Exception:
            return False

    def run_trial(self):
        result = {'request_id': self.identifier, 'status': 'rejected',
                  'requested_distance_m': .4, 'stopped': False}
        try:
            for client in (self.planner, self.nav):
                if not client.wait_for_server(timeout_sec=10):
                    raise RuntimeError('nav2_unavailable')
            for name in ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']:
                client = self.node.create_client(GetState, '/' + name + '/get_state')
                if not client.wait_for_service(timeout_sec=3):
                    raise RuntimeError(name + '_unavailable')
                if self.wait(client.call_async(GetState.Request()), 5).current_state.id != 3:
                    raise RuntimeError(name + '_not_active')
                self.node.destroy_client(client)
            deadline = time.monotonic() + 15
            while not all(k in self.latest for k in ('map', 'scan_at', 'odom_at')) or not self.buffer.can_transform('map', 'vehicle/base_link', rclpy.time.Time()):
                if time.monotonic() > deadline:
                    raise RuntimeError('missing_initial_feedback')
                self.spin_once()
            self.observe(2)
            if any(s.status in (1, 2, 3) for s in self.latest.get('status', SimpleNamespace(status_list=[])).status_list):
                raise RuntimeError('another_navigation_is_active')
            if not self.stop_confirmed():
                raise RuntimeError('robot_not_stationary')
            self.start_pose = self.pose()
            x, y, heading = self.start_pose
            candidate = {'x': x + .4*math.cos(heading), 'y': y + .4*math.sin(heading), 'yaw': heading}
            result['before'] = self.sample()
            result['candidate'] = candidate
            plan, outcome = self.plan(candidate)
            result['planning'] = outcome
            if plan is None:
                raise RuntimeError('path_rejected')
            if plan['length'] > .8:
                raise RuntimeError('planned_detour_exceeds_trial_budget')
            self.health()
            clearance = self.evaluate_path(plan['path'], candidate)
            result['clearance'] = clearance
            if not clearance['safe']:
                raise RuntimeError('path_invalidated_before_dispatch')
            goal = NavigateToPose.Goal()
            goal.pose = self.goal_pose(candidate)
            goal.behavior_tree = str(HERE / 'motion_trial.xml')
            self.active_candidate = candidate
            self.execution_path = plan['path']
            self.execution_started_ns = self.node.get_clock().now().nanoseconds
            self.event('trial_goal_dispatch', request_id=self.identifier, candidate=candidate)
            outcome = self.execute(self.nav, goal, 45, 'navigation')
            result['navigation_result'] = outcome
            result['status'] = 'succeeded' if outcome['status'] == 4 else 'aborted'
        except (Exception, KeyboardInterrupt) as error:
            result['reason'] = str(error) or 'operator_interrupted'
            if self.dispatched:
                result['status'] = 'canceled' if self.cancel_path.exists() else 'aborted'
        finally:
            self.canceling = True
            # If sending timed out, a goal may have been accepted remotely.
            if self.dispatched and self.handle is None and self.pending_goal is not None and 'navigation_result' not in result:
                try:
                    late = self.wait(self.pending_goal, 3)
                    if late.accepted:
                        self.handle = late
                        self.action_result = late.get_result_async()
                except Exception:
                    result['navigation_paused'] = self.pause_navigation()
            result['stopped'] = self.cancel_active()
            if not result['stopped'] and self.dispatched:
                result['navigation_paused'] = self.pause_navigation()
                result['stopped'] = self.stop_confirmed()
                result['status'] = 'stop_unconfirmed' if not result['stopped'] else 'aborted'
            result['distance_odom_m'] = self.distance
            result['minimum_scan_m'] = self.minimum_scan if math.isfinite(self.minimum_scan) else None
            try:
                result['after'] = self.sample()
            except Exception:
                pass
            self.event('trial_finished', **result)
            self.report.update(result)
            self.checkpoint()
            self.event_file.close()
            self.node.destroy_node()
            self.lock.close()
            self.motion_lock.close()
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('request_id')
    args = parser.parse_args()
    identifier = str(uuid.UUID(args.request_id))
    DIRECTORY.mkdir(parents=True, exist_ok=True)
    result_path = DIRECTORY / (identifier + '.result.json')
    if result_path.exists():
        return
    with (DIRECTORY / '.execution.lock').open('w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            result = {'request_id': identifier, 'status': 'rejected', 'reason': 'another_trial_active'}
        else:
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            signal.signal(signal.SIGINT, signal.default_int_handler)
            signal.signal(signal.SIGTERM, signal.default_int_handler)
            try:
                try:
                    trial = Trial(identifier)
                except BlockingIOError:
                    result = {'request_id': identifier, 'status': 'rejected', 'reason': 'another_navigation_owner'}
                else:
                    result = trial.run_trial()
            finally:
                rclpy.shutdown()
        temporary = result_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(result, allow_nan=False, indent=2) + '\n')
        temporary.replace(result_path)


if __name__ == '__main__':
    main()
