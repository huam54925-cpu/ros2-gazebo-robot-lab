#!/usr/bin/env python3
"""Conservative simulation stop guard, not a planner or certified safety system."""
import json
from pathlib import Path
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from safety_profile import FOOTPRINT_MODE, PROFILE, PROFILE_LIMITS, CONTRACT_HASH
from robot_contract import CONTRACT
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from safety_contract import blocked_reason, scan_state


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('velocity_guard',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    stopped = False
    def stop(signum, frame):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    command = Twist()
    command_at = scan_at = command_sim = float('-inf')
    nearest = 0.0
    valid_scan = False
    latest_scan=None
    measured=(0.,0.)
    odom_at=float('-inf')
    last_report=0.
    if FOOTPRINT_MODE:
        from footprint_guard import check_sweep
        from scan_motion import OdomHistory,compensated_points
        history=OdomHistory()
    odom_error=None
    evidence_path=Path(__file__).resolve().parents[1]/'log'/'footprint-guard-status.json'
    def receive_command(msg):
        nonlocal command, command_at,command_sim
        command, command_at = msg, time.monotonic()
        command_sim=node.get_clock().now().nanoseconds*1e-9
    def receive_scan(msg):
        nonlocal scan_at, nearest, valid_scan, latest_scan
        scan_at = time.monotonic()
        nearest, valid_scan = scan_state(msg)
        latest_scan=msg
    def receive_odom(msg):
        nonlocal measured,odom_at,odom_error
        measured=(msg.twist.twist.linear.x,msg.twist.twist.angular.z)
        odom_at=time.monotonic()
        if FOOTPRINT_MODE:
            p=msg.pose.pose.position;q=msg.pose.pose.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            try:
                history.add(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,[p.x,p.y,yaw],msg.header.frame_id)
                odom_error=None
            except ValueError as error:odom_error=str(error)
    subscriptions = [
        node.create_subscription(Odometry, "/model/vehicle/odometry", receive_odom, qos_profile_sensor_data),
        node.create_subscription(Twist, '/model/vehicle/cmd_vel', receive_command, 1),
        node.create_subscription(LaserScan, '/scan', receive_scan, qos_profile_sensor_data),
    ]
    pub = node.create_publisher(Twist, '/model/vehicle/cmd_vel_safe', 1)
    last_reason = ''
    def tick():
        nonlocal last_reason,last_report
        now = time.monotonic()
        reason = blocked_reason(now-command_at, now-scan_at, nearest, valid_scan)
        evidence={}
        if FOOTPRINT_MODE:
            sim=node.get_clock().now().nanoseconds*1e-9
            reason=('command timeout' if now-command_at>CONTRACT['command_wall_timeout_s'] or not 0<=sim-command_sim<=CONTRACT['command_timeout_s'] else
                    'scan missing or stale' if now-scan_at>CONTRACT['feedback_wall_timeout_s'] or not valid_scan else
                    'odometry stale' if now-odom_at>CONTRACT['feedback_wall_timeout_s'] else odom_error or '')
            if not reason:
                try:
                    points,compensation=compensated_points(latest_scan,history,sim)
                    evidence=check_sweep(points,
                        (max(-PROFILE_LIMITS['max_linear_m_s'],min(PROFILE_LIMITS['max_linear_m_s'],command.linear.x)),
                         max(-PROFILE_LIMITS['max_angular_rad_s'],min(PROFILE_LIMITS['max_angular_rad_s'],command.angular.z))),
                        measured,compensation['remaining_pose_age_sim_s'])
                    evidence['motion_compensation']=compensation
                    if not evidence['safe']:reason=evidence['reason']
                except Exception as error:
                    reason='footprint guard invalid input: '+type(error).__name__
        values = [command.linear.x, command.linear.y, command.linear.z,
                  command.angular.x, command.angular.y, command.angular.z]
        if not all(math.isfinite(v) for v in values):
            reason = 'invalid velocity'
        output = Twist()
        if not reason:
            output.linear.x = max(-PROFILE_LIMITS['max_linear_m_s'], min(PROFILE_LIMITS['max_linear_m_s'], command.linear.x))
            output.angular.z = max(-PROFILE_LIMITS['max_angular_rad_s'], min(PROFILE_LIMITS['max_angular_rad_s'], command.angular.z))
        pub.publish(output)
        if FOOTPRINT_MODE and now-last_report>.5:
            temporary=evidence_path.with_suffix('.tmp')
            temporary.write_text(json.dumps({'profile':PROFILE,'contract_hash':CONTRACT_HASH,'generated_monotonic_s':now,
                'reason':reason,'scan_age_s':now-scan_at if latest_scan else None,
                'command':[command.linear.x,command.angular.z],'measured':measured,
                'output':[output.linear.x,output.angular.z],'sweep':evidence}))
            temporary.replace(evidence_path);last_report=now
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
