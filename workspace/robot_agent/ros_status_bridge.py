"""ROS subscriber-only bridge. Run inside the simulation container, without API keys."""
import fcntl
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'navigation_base'))
from safety_contract import STOP_RADIUS_M, blocked_reason, scan_state
from map_identity import map_version
from safety_profile import FOOTPRINT_MODE, PROFILE

OUTPUT = Path(__file__).resolve().parents[1] / 'log' / 'robot-status.json'


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec / 1e9


def pose(position, orientation):
    return {'x': position.x, 'y': position.y, 'z': position.z,
            'yaw_rad': math.atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y),
                                  1 - 2 * (orientation.y**2 + orientation.z**2))}


def velocity(msg):
    return {'linear_x_m_s': msg.linear.x, 'angular_z_rad_s': msg.angular.z}


class StatusBridge:
    def __init__(self):
        self.node = rclpy.create_node('robot_readonly_status', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.latest = {}
        self.nearest, self.valid_scan = math.inf, False
        self.graph = {}
        self.graph_at = float('-inf')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = [
            self.node.create_subscription(ClockMessage, '/clock', self.clock, qos_profile_sensor_data),
            self.node.create_subscription(Odometry, '/model/vehicle/odometry', self.odom, qos_profile_sensor_data),
            self.node.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data),
            self.node.create_subscription(OccupancyGrid, '/map', self.map, latched),
            self.node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.navigation, latched),
            self.node.create_subscription(Twist, '/model/vehicle/cmd_vel',
                                          lambda m: self.save('requested_velocity', velocity(m)), 1),
            self.node.create_subscription(Twist, '/model/vehicle/cmd_vel_safe',
                                          lambda m: self.save('guard_output', velocity(m)), 1),
        ]
        self.timer = self.node.create_timer(0.5, self.write,
                                           clock=Clock(clock_type=ClockType.STEADY_TIME))

    def save(self, name, value):
        self.latest[name] = {**value, 'received_monotonic_s': time.monotonic()}

    def clock(self, msg):
        self.save('clock', {'sim_time_s': msg.clock.sec + msg.clock.nanosec / 1e9})

    def odom(self, msg):
        self.save('odometry', {'frame': msg.header.frame_id, 'child_frame': msg.child_frame_id,
                               'stamp_sim_s': stamp(msg.header),
                               'pose': pose(msg.pose.pose.position, msg.pose.pose.orientation),
                               'velocity': velocity(msg.twist.twist)})

    def scan(self, msg):
        self.nearest, self.valid_scan = scan_state(msg)
        self.save('scan', {'frame': msg.header.frame_id, 'stamp_sim_s': stamp(msg.header),
                           'nearest_m': self.nearest if math.isfinite(self.nearest) else None,
                           'valid': self.valid_scan, 'sample_count': len(msg.ranges),
                           'no_finite_hit': not math.isfinite(self.nearest)})

    def map(self, msg):
        cells = msg.data
        unknown = sum(v < 0 for v in cells)
        free = sum(v == 0 for v in cells)
        digest = hashlib.sha256(bytes((v & 255 for v in cells))).hexdigest()[:16]
        self.save('map', {'frame': msg.header.frame_id, 'stamp_sim_s': stamp(msg.header),
                          'width': msg.info.width, 'height': msg.info.height,
                          'resolution_m': msg.info.resolution,
                          'origin': pose(msg.info.origin.position, msg.info.origin.orientation),
                          'content_hash': digest, 'map_version': map_version(msg), 'unknown_cells': unknown,
                          'free_cells': free, 'other_known_cells': len(cells)-unknown-free,
                          'known_area_m2': (len(cells)-unknown)*msg.info.resolution**2,
                          'coverage': None})

    def navigation(self, msg):
        self.save('navigation', {'statuses': [
            {'goal_id': bytes(item.goal_info.goal_id.uuid).hex(), 'status_code': item.status}
            for item in msg.status_list[-16:]],
            'active_goal_count': sum(item.status in (1, 2, 3) for item in msg.status_list)})

    def write(self):
        now = time.monotonic()
        if now - self.graph_at >= 3:
            names = self.node.get_node_names()
            services = dict(self.node.get_service_names_and_types())
            self.graph = {'guard_node_present': 'velocity_guard' in names,
                          'navigation_action_server_present':
                              '/navigate_to_pose/_action/get_result' in services,
                          'observed_monotonic_s': now}
            self.graph_at = now
        try:
            transform = self.buffer.lookup_transform('map', 'vehicle/base_link', rclpy.time.Time())
            self.save('map_pose', {'frame': 'map', 'child_frame': 'vehicle/base_link',
                                   'stamp_sim_s': stamp(transform.header),
                                   'pose': pose(transform.transform.translation,
                                                transform.transform.rotation)})
        except TransformException:
            pass
        cmd_age = now - self.latest.get('requested_velocity', {}).get('received_monotonic_s', float('-inf'))
        scan_age = now - self.latest.get('scan', {}).get('received_monotonic_s', float('-inf'))
        result = {'schema_version': 1, 'generated_monotonic_s': now,
                  'generated_unix_s': time.time(), 'mode': 'read_only',
                  'sources': self.latest, 'graph': self.graph,
                  'guard': {'stop_radius_m': STOP_RADIUS_M,
                            'reason_inferred_from_inputs': blocked_reason(cmd_age, scan_age,
                                                                        self.nearest, self.valid_scan),
                            'authoritative_guard_status': False}}
        if FOOTPRINT_MODE:
            try:
                guard=json.loads((OUTPUT.parent/'footprint-guard-status.json').read_text())
                guard['fresh']=now-guard['generated_monotonic_s']<2.
            except (OSError,ValueError):guard={'fresh':False}
            result['guard']={'profile':PROFILE,'stop_radius_m':None,'body_margin_m':.075,
                             'live_guard':guard,'authoritative_guard_status':guard['fresh']}
        # Reject invalid numeric data instead of writing nonstandard NaN/Infinity JSON.
        try:
            encoded = json.dumps(result, allow_nan=False)
        except ValueError:
            self.node.get_logger().error('Non-finite state: snapshot not updated')
            return
        temporary = OUTPUT.with_suffix('.tmp')
        temporary.write_text(encoded + '\n')
        temporary.replace(OUTPUT)


def main():
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT.with_suffix('.lock').open('w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('Read-only bridge is already running')
        rclpy.init()
        bridge = StatusBridge()
        try:
            rclpy.spin(bridge.node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            bridge.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
