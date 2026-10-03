#!/usr/bin/env python3
"""Check live map contents and scan-time TF without moving the simulation."""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = Node('check_slam')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    messages = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(OccupancyGrid, '/map', lambda m: messages.update(map=m), qos),
        node.create_subscription(LaserScan, '/scan', lambda m: messages.setdefault('scan', m),
                                 qos_profile_sensor_data),
    ]
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    deadline = time.monotonic() + 45
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if 'map' not in messages or 'scan' not in messages:
                continue
            scan = messages['scan']
            stamp = rclpy.time.Time.from_msg(scan.header.stamp)
            if not buffer.can_transform('map', scan.header.frame_id, stamp):
                # Keep a recent scan so the first sample predating TF discovery cannot block.
                messages.pop('scan')
                continue
            m = messages['map']
            free = sum(0 <= v < 25 for v in m.data)
            occupied = sum(v >= 65 for v in m.data)
            assert m.header.frame_id == 'map'
            assert len(m.data) == m.info.width * m.info.height > 0
            assert free > 0 and occupied > 0, 'Map must contain free and occupied cells'
            tf = buffer.lookup_transform('map', 'vehicle/odom', rclpy.time.Time())
            print(json.dumps({'passed': True, 'map_frame': m.header.frame_id,
                'width': m.info.width, 'height': m.info.height,
                'resolution': m.info.resolution, 'free_cells': free,
                'occupied_cells': occupied, 'unknown_cells': sum(v < 0 for v in m.data),
                'map_stamp_sec': m.header.stamp.sec,
                'scan_time_tf_available': True,
                'map_to_odom_translation': [tf.transform.translation.x,
                                           tf.transform.translation.y,
                                           tf.transform.translation.z]}, indent=2))
            return
        raise RuntimeError('Timed out waiting for a nonempty map and scan-time map-to-lidar TF')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
