#!/usr/bin/env python3
"""Move this simulated vehicle briefly, always stop, and verify feedback/TF."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

rclpy.init()
n = Node('verify_vehicle_motion')
n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
latest = {}
def save(key):
    return lambda msg: latest.update({key: msg})
subs = [n.create_subscription(Odometry, '/model/vehicle/odometry', save('odom'), 10),
        n.create_subscription(JointState, '/joint_states', save('joints'), 10),
        n.create_subscription(String, '/robot_description', save('description'),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
buf = Buffer(); listener = TransformListener(buf, n)
pub = n.create_publisher(Twist, '/model/vehicle/cmd_vel', 10)
def pump(seconds, command=None):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if command is not None: pub.publish(command)
        rclpy.spin_once(n, timeout_sec=0.1)
def yaw(o):
    q = o.pose.pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
result = {}
try:
    deadline = time.monotonic()+30
    while (not all(k in latest for k in ('odom','joints','description')) or not pub.get_subscription_count()) and time.monotonic()<deadline:
        rclpy.spin_once(n, timeout_sec=0.2)
    assert all(k in latest for k in ('odom','joints','description')), 'Missing ROS feedback or robot_description'
    initial = latest['odom']
    cmd = Twist(); cmd.linear.x = 0.2; pump(4, cmd)
    forward = latest['odom']
    cmd = Twist(); cmd.angular.z = 0.3; pump(4, cmd)
    turned = latest['odom']
    pump(2, Twist())
    stopped = latest['odom']
    result['odom_frames'] = [stopped.header.frame_id, stopped.child_frame_id]
    result['forward_distance_m'] = math.hypot(forward.pose.pose.position.x-initial.pose.pose.position.x, forward.pose.pose.position.y-initial.pose.pose.position.y)
    result['turn_radians'] = abs(math.atan2(math.sin(yaw(turned)-yaw(forward)),math.cos(yaw(turned)-yaw(forward))))
    result['stopped_velocity'] = [stopped.twist.twist.linear.x, stopped.twist.twist.angular.z]
    result['joint_names'] = latest['joints'].name
    result['tf'] = {}
    for frame in ['chassis','left_wheel','right_wheel','caster']:
        t = buf.lookup_transform('vehicle/odom','vehicle/'+frame,rclpy.time.Time())
        result['tf'][frame] = {'stamp_sec':t.header.stamp.sec,'xyz':[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]}
    assert result['odom_frames'] == ['vehicle/odom','vehicle/chassis']
    assert result['forward_distance_m'] > 0.05, 'No forward feedback'
    assert result['turn_radians'] > 0.05, 'No turn feedback'
    assert all(abs(v)<0.02 for v in result['stopped_velocity']), 'Vehicle did not stop'
    result['passed'] = True
finally:
    pump(1,Twist())
    print(json.dumps(result,indent=2),flush=True)
    n.destroy_node(); rclpy.shutdown()
