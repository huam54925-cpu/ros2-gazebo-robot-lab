#!/usr/bin/env python3
"""Opt-in Gazebo REP-117 no-return adapter for SLAM only; never /scan or cmd_vel.

Karto ignores ranges >= range_max, while ranges >= its range threshold and
< range_max clear rays without marking an occupied endpoint. The paired
experimental mapper must use max_laser_range=11.9 for this 12 m sensor.
NaN, negative infinity and all finite readings are preserved, not invented.
"""
import argparse
import copy
import json
import math
import struct
import time

MAPPING_RANGE_M = 11.9
SENSOR_RANGE_M = 12.0


def adapt_ranges(ranges, range_min, range_max, frame):
    if frame != 'vehicle/lidar' or not math.isfinite(range_min) or not 0 < range_min < MAPPING_RANGE_M:
        raise ValueError('unexpected_sensor_contract')
    if range_max != SENSOR_RANGE_M or len(ranges) != 360:
        raise ValueError('unexpected_sensor_contract')
    # Next lower IEEE float32 value survives LaserScan serialization below max.
    bits=struct.unpack('<I',struct.pack('<f',range_max))[0]
    clearing=struct.unpack('<f',struct.pack('<I',bits-1))[0]
    assert MAPPING_RANGE_M < clearing < range_max
    return [clearing if v == math.inf else v for v in ranges]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--contract',required=True,choices=['gazebo_rep117_12m_360'])
    args=parser.parse_args()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    rclpy.init();node=rclpy.create_node('gazebo_scan_slam_adapter')
    pub=node.create_publisher(LaserScan,'/scan_slam_finite',qos_profile_sensor_data)
    counts={'messages':0,'converted_positive_inf':0,'rejected_messages':0}
    last=[0.]
    def callback(scan):
        try:
            ranges=adapt_ranges(scan.ranges,scan.range_min,scan.range_max,scan.header.frame_id)
        except ValueError:
            counts['rejected_messages']+=1
            return
        msg=copy.deepcopy(scan);msg.ranges=ranges;pub.publish(msg)
        counts['messages']+=1;counts['converted_positive_inf']+=sum(v==math.inf for v in scan.ranges)
        if time.monotonic()-last[0]>20:
            print(json.dumps({**counts,'contract':args.contract,'mapping_range_m':MAPPING_RANGE_M,
                              'input':'/scan','output':'/scan_slam_finite','original_stamp_preserved':True}),flush=True)
            last[0]=time.monotonic()
    sub=node.create_subscription(LaserScan,'/scan',callback,qos_profile_sensor_data)
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
