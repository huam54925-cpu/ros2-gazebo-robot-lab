#!/usr/bin/env python3
"""Read-only startup check: scan, odometry, clock and lidar TF; does not drive."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = Node('check_mapping_scene')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    messages = {}
    counts = {'scan': 0, 'odom': 0, 'clock': 0}
    def receive(key):
        def callback(message):
            messages[key] = message
            counts[key] += 1
        return callback
    subscriptions = [
        node.create_subscription(LaserScan, '/scan', receive('scan'), qos_profile_sensor_data),
        node.create_subscription(Odometry, '/model/vehicle/odometry', receive('odom'), 10),
        node.create_subscription(Clock, '/clock', receive('clock'), qos_profile_sensor_data),
    ]
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    deadline = time.monotonic() + 45
    ready = False
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            ready = (all(n >= 3 for n in counts.values()) and buffer.can_transform(
                'vehicle/odom', 'vehicle/lidar', rclpy.time.Time()))
            if ready:
                break
        if not ready:
            raise RuntimeError(f'Feedback or TF missing: {counts}')
        scan, odom = messages['scan'], messages['odom']
        finite = [v for v in scan.ranges if math.isfinite(v) and scan.range_min <= v <= scan.range_max]
        if scan.header.frame_id != 'vehicle/lidar' or len(scan.ranges) != 360 or len(finite) < 90:
            raise RuntimeError('Unexpected laser scan contents')
        if (odom.header.frame_id, odom.child_frame_id) != ('vehicle/odom', 'vehicle/base_link'):
            raise RuntimeError('Unexpected odometry frames')
        tf = buffer.lookup_transform('vehicle/chassis', 'vehicle/lidar', rclpy.time.Time())
        if abs(tf.transform.translation.z - 0.4) > 1e-5:
            raise RuntimeError('Unexpected lidar height')
        print(json.dumps({'passed': True, 'message_counts': counts,
            'scan_frame': scan.header.frame_id, 'rays': len(scan.ranges),
            'finite_returns': len(finite), 'nearest_m': min(finite), 'farthest_m': max(finite),
            'odom_frames': [odom.header.frame_id, odom.child_frame_id],
            'lidar_height_above_chassis_m': tf.transform.translation.z,
            'sim_time_sec': messages['clock'].clock.sec}, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
