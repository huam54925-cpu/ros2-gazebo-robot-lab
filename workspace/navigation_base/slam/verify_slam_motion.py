#!/usr/bin/env python3
"""Drive a 1 m square in the indoor scene's clear spawn area, then stop.

Run only from the initial indoor spawn pose, with SLAM active. This is a short
mapping regression, not an autonomous planner or a full-room mapping route.
"""
import json
import math
import time
import signal
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    # Keep DDS alive until the finally block has sent stop commands.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = Node('verify_slam_motion')
    latest = {}
    def odom(m):
        latest['odom'] = m
        latest['receipt'] = time.monotonic()
    subs = [node.create_subscription(Odometry, '/model/vehicle/odometry', odom, 10),
            node.create_subscription(OccupancyGrid, '/map', lambda m: latest.update(map=m),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    pub = node.create_publisher(Twist, '/model/vehicle/cmd_vel', 10)
    result = {'passed': False}
    def spin():
        rclpy.spin_once(node, timeout_sec=0.1)
    def pose():
        p = latest['odom'].pose.pose
        q = p.orientation
        return p.position.x, p.position.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    def wrap(a):
        return math.atan2(math.sin(a), math.cos(a))
    try:
        deadline = time.monotonic()+30
        while not ('odom' in latest and 'map' in latest and pub.get_subscription_count()):
            if time.monotonic()>deadline:
                raise RuntimeError('Missing odom, map or command subscriber')
            spin()
        x,y,yaw=pose()
        assert math.hypot(x,y)<0.2 and abs(yaw)<0.1, 'Run only at the indoor spawn pose'
        initial_map=latest['map']
        initial_known=sum(v>=0 for v in initial_map.data)
        reached=[]
        deadline=time.monotonic()+300
        for gx,gy in [(1.,0.),(1.,1.),(0.,1.),(0.,0.)]:
            while True:
                spin()
                if time.monotonic()>deadline or time.monotonic()-latest['receipt']>3:
                    raise RuntimeError('Motion deadline exceeded or odometry is stale')
                x,y,yaw=pose()
                distance=math.hypot(gx-x,gy-y)
                if distance<0.08:
                    pub.publish(Twist())
                    reached.append([x,y])
                    print(json.dumps({'waypoint_reached': [gx,gy], 'actual': [x,y]}),flush=True)
                    break
                error=wrap(math.atan2(gy-y,gx-x)-yaw)
                cmd=Twist()
                cmd.angular.z=max(-0.35,min(0.35,1.2*error))
                if abs(error)<0.15:
                    cmd.linear.x=min(0.25,max(0.06,0.6*distance))
                pub.publish(cmd)
        # Return to the initial heading, then allow the map to update.
        while True:
            spin()
            if time.monotonic()>deadline or time.monotonic()-latest['receipt']>3:
                raise RuntimeError('Timed out restoring heading')
            x,y,yaw=pose()
            if abs(yaw)<0.04:
                break
            cmd=Twist(); cmd.angular.z=max(-0.35,min(0.35,-yaw)); pub.publish(cmd)
        until=time.monotonic()+12
        while time.monotonic()<until:
            pub.publish(Twist()); spin()
        m=latest['map']
        v=latest['odom'].twist.twist
        known=sum(v>=0 for v in m.data)
        assert known>initial_known, 'Mapped area did not grow'
        assert abs(v.linear.x)<0.02 and abs(v.angular.z)<0.02, 'Vehicle did not stop'
        result.update(passed=True, initial_known_cells=initial_known, final_known_cells=known,
                      reached=reached, final_pose=list(pose()),
                      stopped_velocity=[v.linear.x,v.angular.z])
    except KeyboardInterrupt:
        result['interrupted'] = True
    finally:
        until=time.monotonic()+1
        while time.monotonic()<until:
            pub.publish(Twist()); time.sleep(0.05)
        print(json.dumps(result,indent=2),flush=True)
        node.destroy_node(); rclpy.shutdown()


if __name__=='__main__':
    main()
