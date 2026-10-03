#!/usr/bin/env python3
"""Conservative simulation stop guard, not a planner or certified safety system."""
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions


def blocked_reason(command_age, scan_age, nearest, valid_scan):
    if command_age > 0.5:
        return 'command timeout'
    if scan_age > 1.5 or not valid_scan:
        return 'scan missing or stale'
    if nearest < 1.9:
        return 'obstacle inside 1.9 m stop radius'
    return ''


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('velocity_guard')
    stopped = False
    def stop(signum, frame):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    command = Twist()
    command_at = scan_at = float('-inf')
    nearest = 0.0
    valid_scan = False
    def receive_command(msg):
        nonlocal command, command_at
        command, command_at = msg, time.monotonic()
    def receive_scan(msg):
        nonlocal scan_at, nearest, valid_scan
        scan_at = time.monotonic()
        valid_scan = msg.header.frame_id == 'vehicle/lidar' and len(msg.ranges) > 0
        returns = [v for v in msg.ranges if math.isfinite(v) and msg.range_min <= v <= msg.range_max]
        valid_scan = valid_scan and (bool(returns) or any(v == math.inf for v in msg.ranges))
        nearest = min(returns, default=math.inf)
    subscriptions = [
        node.create_subscription(Twist, '/model/vehicle/cmd_vel', receive_command, 1),
        node.create_subscription(LaserScan, '/scan', receive_scan, qos_profile_sensor_data),
    ]
    pub = node.create_publisher(Twist, '/model/vehicle/cmd_vel_safe', 1)
    last_reason = ''
    def tick():
        nonlocal last_reason
        now = time.monotonic()
        reason = blocked_reason(now-command_at, now-scan_at, nearest, valid_scan)
        values = [command.linear.x, command.linear.y, command.linear.z,
                  command.angular.x, command.angular.y, command.angular.z]
        if not all(math.isfinite(v) for v in values):
            reason = 'invalid velocity'
        output = Twist()
        if not reason:
            output.linear.x = max(-0.25, min(0.25, command.linear.x))
            output.angular.z = max(-0.35, min(0.35, command.angular.z))
        pub.publish(output)
        if reason and reason != last_reason:
            node.get_logger().info('Stopped: ' + reason)
        last_reason = reason
    timer = node.create_timer(0.05, tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
    try:
        while not stopped:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        for _ in range(10):
            pub.publish(Twist())
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
