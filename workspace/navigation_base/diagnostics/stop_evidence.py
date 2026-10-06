#!/usr/bin/env python3
"""Read-only stop evidence recorder; never publishes a robot command.

Distances are measured lidar returns to the unpadded contract body, not ground
truth. Nav2 logs do not expose their collision witness: keep it explicitly null.
Snapshots retain timestamps and errors rather than claiming exact simultaneity.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys
import time
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from footprint_guard import check_sweep
from scan_motion import OdomHistory, compensated_points
from robot_contract import CONTRACT
from observation import navigation_footprint, polygon_square_distances


def nearest_body_return(points):
    if not len(points):return None
    body, _ = navigation_footprint()
    distances = polygon_square_distances(body, points, 0.)
    i = int(np.argmin(distances))
    return {'hit_base_xy': points[i].tolist(),
            'distance_to_unpadded_body_m': float(distances[i]),
            'source': 'motion_compensated_lidar; not ground_truth'}


def main():
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from rcl_interfaces.msg import Log
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    output = Path(args.output); output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('stop_evidence_observer', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=True)])
    history = OdomHistory(); scans = deque(maxlen=20); ring = deque(maxlen=30)
    command = [0., 0.]; safe = [0., 0.]; measured = [0., 0.]
    state = {'command_wall_s': None, 'safe_wall_s': None, 'odom_error': None}
    guard_path = Path(__file__).resolve().parents[2]/'log'/'footprint-guard-status.json'
    previous = {'moving': False, 'reason': None, 'last_log': None, 'sample_wall_s': 0.}
    def odom(msg):
        p = msg.pose.pose.position; q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        try:
            history.add(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                        [p.x, p.y, yaw], msg.header.frame_id)
            state['odom_error'] = None
        except ValueError as error:state['odom_error'] = str(error)
        measured[:] = [msg.twist.twist.linear.x, msg.twist.twist.angular.z]
    def velocity(msg, target, key):
        target[:] = [msg.linear.x, msg.angular.z]; state[key] = time.time()
    def save(component, reason, extra=None):
        row = {'wall_unix_s': time.time(), 'trigger_component': component,
               'reason': reason, 'component_collision_witness': None,
               'note': 'Observed stop/log trigger; no inferred global blockage.',
               'context': list(ring), 'extra': extra}
        with output.open('a') as stream:stream.write(json.dumps(row)+'\n')
    def log(msg):
        text = msg.msg
        if not any(word in text.lower() for word in ('collision', 'failed', 'no valid', 'stopped:', 'aborting')):return
        identity = (msg.name, text)
        if identity == previous['last_log']:return
        previous['last_log'] = identity
        save(msg.name, text, {'ros_log_stamp': [msg.stamp.sec, msg.stamp.nanosec]})
    def sample():
        now = time.time(); sim = node.get_clock().now().nanoseconds*1e-9
        if not any(abs(value)>1e-4 for value in [*command,*safe,*measured]) and now-previous['sample_wall_s']<1.:return
        previous['sample_wall_s'] = now
        row = {'wall_unix_s': now, 'simulation_seconds': sim, 'command': command[:],
               'guarded_command': safe[:], 'measured': measured[:], **state,
               'footprint': CONTRACT['footprint'], 'body_padding_m': CONTRACT['body_padding_m']}
        if history.rows:row['odom'] = list(history.rows[-1])
        # Use the newest scan whose acquisition interval is actually bracketed.
        # Retain its stamp/age, and report absence instead of extrapolating.
        error = 'no_synchronized_scan'
        for index, scan in enumerate(reversed(scans)):
            if index == 0:
                row['latest_scan_stamp_sim_s'] = scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
                row['latest_scan_compensation_error'] = None
            try:
                points, metadata = compensated_points(scan, history, sim)
                row['scan'] = metadata
                row['nearest_body_return'] = nearest_body_return(points)
                row['independent_sweep_replay'] = check_sweep(points, command, measured,
                    metadata['remaining_pose_age_sim_s'])
                error = None; break
            except ValueError as exception:
                error = str(exception)
                if index == 0:row['latest_scan_compensation_error'] = error
        row['scan_error'] = error
        try:
            guard = json.loads(guard_path.read_text())
            row['live_guard'] = guard
            row['guard_evidence_age_wall_s'] = time.monotonic()-guard['generated_monotonic_s']
        except (OSError, ValueError, KeyError):guard = {}
        ring.append(row)
        reason = guard.get('reason')
        if reason and reason != previous['reason']:save('velocity_guard', reason)
        previous['reason'] = reason
        moving = abs(safe[0])+abs(safe[1]) > 1e-4
        if previous['moving'] and not moving:save('command_chain', 'guarded_command_became_zero')
        previous['moving'] = moving
    subscriptions = [
        node.create_subscription(LaserScan, '/scan', scans.append, qos_profile_sensor_data),
        node.create_subscription(Odometry, '/model/vehicle/odometry', odom, qos_profile_sensor_data),
        node.create_subscription(Twist, '/model/vehicle/cmd_vel',
            lambda msg: velocity(msg, command, 'command_wall_s'), 1),
        node.create_subscription(Twist, '/model/vehicle/cmd_vel_safe',
            lambda msg: velocity(msg, safe, 'safe_wall_s'), 1),
        node.create_subscription(Log, '/rosout', log, 100)]
    timer = node.create_timer(.2, sample, clock=Clock(clock_type=ClockType.STEADY_TIME))
    try:rclpy.spin(node)
    finally:node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':main()
