#!/usr/bin/env python3
"""Integration test; run ONLY in an isolated --network none container."""
import os
import signal
import subprocess
import time
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy

rclpy.init()
n=rclpy.create_node('stop_regression')
root=Path(__file__).resolve().parents[1]
scan_pub=n.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
cmd_pub=n.create_publisher(Twist,'/model/vehicle/cmd_vel',1)
latest={}
subs=[n.create_subscription(Twist,'/model/vehicle/cmd_vel_safe',lambda m:latest.update(safe=m.linear.x),1),
      n.create_subscription(Twist,'/model/vehicle/cmd_vel',lambda m:latest.update(raw=m.linear.x),1)]
scan=LaserScan();scan.header.frame_id='vehicle/lidar';scan.range_min=0.1;scan.range_max=12.;scan.ranges=[5.]*360
cmd=Twist();cmd.linear.x=.2
children=[]
def pump(seconds, scans=False, commands=False, feedback=None):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if scans:scan_pub.publish(scan)
        if commands:cmd_pub.publish(cmd)
        if feedback:feedback()
        rclpy.spin_once(n,timeout_sec=.03)
        time.sleep(.02)
try:
    guard=subprocess.Popen(['python3',str(root/'velocity_guard.py')],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    children.append(guard)
    pump(3,scans=True,commands=True)
    assert latest.get('safe',0)>.1, latest
    pump(.9,scans=True)
    assert latest['safe']==0, 'Command loss did not stop'
    pump(.6,scans=True,commands=True)
    assert latest['safe']>.1
    scan.ranges=[1.]*360;pump(.6,scans=True,commands=True)
    assert latest['safe']==0, 'Nearby obstacle did not stop'
    scan.ranges=[5.]*360;pump(.6,scans=True,commands=True)
    assert latest['safe']>.1
    pump(1.9,commands=True)
    assert latest['safe']==0, 'Stale laser did not stop'
    pump(.6,scans=True,commands=True)
    guard.send_signal(signal.SIGTERM);pump(1)
    assert latest['safe']==0, 'Guard termination did not stop'
    assert guard.wait(timeout=3)==0
    print('PASS: command loss, near obstacle, stale laser and guard termination',flush=True)

    odom_pub=n.create_publisher(Odometry,'/model/vehicle/odometry',10)
    map_pub=n.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    odom=Odometry();odom.pose.pose.orientation.w=1.
    m=OccupancyGrid();m.info.width=1;m.info.height=1;m.data=[0]
    def feedback():odom_pub.publish(odom);map_pub.publish(m)
    demo=subprocess.Popen(['python3',str(root/'slam/verify_slam_motion.py')],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    children.append(demo)
    pump(3,feedback=feedback)
    assert latest.get('raw',0)>.1, 'Demo never started'
    demo.send_signal(signal.SIGINT);pump(1.5,feedback=feedback)
    output=demo.communicate(timeout=3)[0]
    assert demo.returncode==0 and '"interrupted": true' in output,output
    assert latest['raw']==0, 'SIGINT demo exit failed to send zero command'
    print('PASS: interrupt during motion publishes stop before ROS shutdown',flush=True)
finally:
    for child in children:
        if child.poll() is None:
            child.terminate()
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:child.kill();child.wait()
    n.destroy_node();rclpy.shutdown()
