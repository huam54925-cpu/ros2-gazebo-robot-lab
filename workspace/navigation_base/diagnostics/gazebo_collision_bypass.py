#!/usr/bin/env python3
"""Explicit Gazebo-only operator relay; command/odometry loss still stops motion.

Pause the existing guard instead of exiting it (exit shuts down Gazebo). Restore
the guard and Nav2 collision checking when this expiring diagnostic session ends.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from simulation_override import MARKER, active
from robot_contract import CONTRACT, CONTRACT_HASH


def main():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rcl_interfaces.srv import SetParameters
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--guard-pid',type=int,required=True)
    parser.add_argument('--seconds',type=float,default=1800.)
    args=parser.parse_args()
    if not Path('/.dockerenv').exists() or not 0<args.seconds<=3600:
        raise RuntimeError('explicit_current_docker_simulation_only')
    commandline=Path(f'/proc/{args.guard_pid}/cmdline').read_bytes()
    if b'/navigation_base/velocity_guard.py' not in commandline:
        raise RuntimeError('unexpected_guard_process')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('gazebo_diagnostic_relay',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    stopped=[False]
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:stopped.__setitem__(0,True))
    pub=node.create_publisher(Twist,'/model/vehicle/cmd_vel_safe',1)
    client=node.create_client(SetParameters,'/controller_server/set_parameters')
    def collision_check(enabled):
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('controller_parameters_unavailable')
        req=SetParameters.Request()
        req.parameters=[rclpy.parameter.Parameter('FollowPath.use_collision_detection',value=enabled).to_parameter_msg()]
        future=client.call_async(req);deadline=time.monotonic()+5
        while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
        if not future.done() or not all(r.successful for r in future.result().results):
            raise RuntimeError('controller_collision_toggle_failed')
    latest={'command':Twist(),'command_wall':-math.inf,'command_sim':-math.inf,'odom_wall':-math.inf}
    def receive(msg):
        latest.update(command=msg,command_wall=time.monotonic(),command_sim=node.get_clock().now().nanoseconds*1e-9)
    def odom(msg):latest['odom_wall']=time.monotonic()
    subscriptions=[node.create_subscription(Twist,'/model/vehicle/cmd_vel',receive,1),
        node.create_subscription(Odometry,'/model/vehicle/odometry',odom,qos_profile_sensor_data)]
    paused=False;changed=False;last_report=0.
    try:
        collision_check(False);changed=True
        record={'mode':'gazebo_collision_bypass','enabled':True,'hostname':os.uname().nodename,
                'started_unix_s':time.time(),'expires_unix_s':time.time()+args.seconds,
                'operator_request':'Pause collision/safety blocking for continuous Gazebo exploration',
                'guard_pid':args.guard_pid,'collision_results_valid_for_normal_mode':False}
        MARKER.write_text(json.dumps(record,indent=2))
        os.kill(args.guard_pid,signal.SIGSTOP);paused=True
        print(json.dumps({'status':'active',**record}),flush=True)
        while not stopped[0] and active():
            rclpy.spin_once(node,timeout_sec=.025)
            now=time.monotonic();sim=node.get_clock().now().nanoseconds*1e-9
            source=latest['command'];output=Twist();reason=''
            if (now-latest['command_wall']>5 or not 0<=sim-latest['command_sim']<=.5):reason='command_timeout'
            elif now-latest['odom_wall']>5:reason='odometry_unavailable'
            elif not all(math.isfinite(v) for v in [source.linear.x,source.angular.z]):reason='invalid_command'
            else:
                limits=CONTRACT['profiles']['footprint_075']
                output.linear.x=max(-limits['max_linear_m_s'],min(limits['max_linear_m_s'],source.linear.x))
                output.angular.z=max(-limits['max_angular_rad_s'],min(limits['max_angular_rad_s'],source.angular.z))
            pub.publish(output)
            if now-last_report>.5:
                path=MARKER.with_name('footprint-guard-status.json');tmp=path.with_suffix('.tmp')
                tmp.write_text(json.dumps({'profile':'footprint_075','contract_hash':CONTRACT_HASH,
                    'generated_monotonic_s':now,'reason':reason,'collision_checks':False,
                    'mode':'gazebo_collision_bypass','command':[source.linear.x,source.angular.z],
                    'output':[output.linear.x,output.angular.z],'sweep':{'safe':None,'reason':'operator_suspended'}}))
                tmp.replace(path);last_report=now
    finally:
        if MARKER.exists():
            record=json.loads(MARKER.read_text());record['enabled']=False;MARKER.write_text(json.dumps(record,indent=2))
        for _ in range(5):pub.publish(Twist());time.sleep(.05)
        if changed:
            try:collision_check(True)
            except Exception as error:print(str(error),flush=True)
        if paused:os.kill(args.guard_pid,signal.SIGCONT)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
