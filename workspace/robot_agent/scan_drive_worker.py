"""V2 direct Gazebo motion primitives with an optional collision mount."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from scan_drive_geometry import wrap,scan_rays,direction_summary,half_range_goal
from safety_mode import runtime_policy,load_mount


def main():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry,OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer,TransformListener
    parser=argparse.ArgumentParser();parser.add_argument('--request',required=True);args=parser.parse_args()
    request=json.loads(Path(args.request).read_text());directory=Path(request['directory'])
    directory.mkdir(parents=True,exist_ok=True)
    output=directory/(request['action_id']+'.result.json')
    if not Path('/.dockerenv').exists():
        raise RuntimeError('gazebo_container_required')
    policy=runtime_policy();mount=load_mount(policy)
    if request.get('safety_mount') != policy:
        raise RuntimeError('safety_mount_policy_mismatch')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('scan_drive_executor',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    buffer=Buffer();listener=TransformListener(buffer,node);latest={};stopped=[False]
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,lambda *_:stopped.__setitem__(0,True))
    def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def odom(msg):
        p=msg.pose.pose
        latest['odom']=[p.position.x,p.position.y,yaw(p.orientation)]
        latest['velocity']=[msg.twist.twist.linear.x,msg.twist.twist.angular.z]
        latest['odom_stamp']=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    def scan(msg):
        latest['scan']=msg
        latest['scan_odom']=list(latest['odom']) if 'odom' in latest else None
    def grid(msg):latest['map']=msg
    subscriptions=[node.create_subscription(Odometry,'/model/vehicle/odometry',odom,qos_profile_sensor_data),
        node.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data),
        node.create_subscription(OccupancyGrid,'/map',grid,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    pub=node.create_publisher(Twist,'/model/vehicle/cmd_vel_safe',1)
    started=time.monotonic();deadline=started+request['wall_remaining_s'];last_trace=0.;distance=0.;previous=None
    phase='initializing';tracefile=directory/(request['action_id']+'.trace.jsonl')
    report={'action_id':request['action_id'],'action':request['action'],'status':'running',
            'collision_checks':policy['enabled'],'safety_mount':policy,'traditional_algorithm':False,'controller':'direct_odometry_feedback',
            'started_unix_s':time.time(),'operating_speeds':{'linear_m_s':.6,'angular_rad_s':.8}}
    def map_pose():
        t=buffer.lookup_transform('map','vehicle/base_link',rclpy.time.Time()).transform
        return [t.translation.x,t.translation.y,yaw(t.rotation)]
    def sample():
        return {'wall_unix_s':time.time(),'simulation_seconds':node.get_clock().now().nanoseconds*1e-9,
                'phase':phase,'odom_pose':latest.get('odom'),'velocity':latest.get('velocity'),
                'distance_odom_m':distance}
    def spin(check_deadline=True):
        nonlocal previous,distance,last_trace
        rclpy.spin_once(node,timeout_sec=.01)
        if 'odom' in latest:
            if previous is not None:distance+=math.dist(previous[:2],latest['odom'][:2])
            previous=list(latest['odom'])
        if stopped[0] or (directory/'STOP').exists():raise RuntimeError('operator_stop')
        if check_deadline and time.monotonic()>=deadline:raise RuntimeError('run_wall_budget')
        if time.monotonic()-last_trace>.2:
            with tracefile.open('a') as f:f.write(json.dumps(sample())+'\n')
            last_trace=time.monotonic()
    def command(v=0.,w=0.):
        if mount is not None and (v or w):
            evidence=mount.evaluate(latest['scan'],latest['odom'],latest.get('scan_odom'),node.get_clock().now().nanoseconds*1e-9,v,w)
            if not evidence['allowed']:
                report['mount_rejection']=evidence
                pub.publish(Twist())
                raise RuntimeError('safety_mount:'+evidence['reason'])
        msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);pub.publish(msg)
    def settle():
        until=time.monotonic()+.6
        while time.monotonic()<until:
            command();rclpy.spin_once(node,timeout_sec=.02)
        command()
    def turn(angle):
        nonlocal phase
        phase='scanning' if abs(angle)>6 else 'turning'
        previous_yaw=latest['odom'][2];turned=0.
        while abs(angle-turned)>.025:
            spin();current=latest['odom'][2];turned+=wrap(current-previous_yaw);previous_yaw=current
            remaining=angle-turned
            if abs(remaining)<=.025:break
            command(0.,math.copysign(min(.8,max(.08,abs(remaining)*1.8)),remaining))
            time.sleep(.02)
        settle();return turned
    def snapshot():
        from map_view import render
        m=latest['map'];o=m.info.origin;data=np.asarray(m.data).reshape(m.info.height,m.info.width)
        g={'frame':'map','data':data,'resolution':m.info.resolution,
           'origin':[o.position.x,o.position.y,yaw(o.orientation)],
           'stamp_sim_s':m.header.stamp.sec+m.header.stamp.nanosec*1e-9}
        pose=map_pose();version=hashlib.sha256(data.tobytes()).hexdigest()[:20]
        view=render(g,pose,'direct_scan_drive',version,[], 'optional_lidar_sweep' if policy['enabled'] else 'collision_checks_disabled')
        raw=latest['scan'];rays=scan_rays(raw.ranges,raw.angle_min,raw.angle_increment,raw.range_min,raw.range_max,pose[2])
        np.savez_compressed(directory/(request['action_id']+'.map.npz'),data=data,origin=g['origin'],resolution=g['resolution'])
        import base64
        (directory/(request['action_id']+'.map.png')).write_bytes(base64.b64decode(view['image']['base64']))
        return {'pose_map':pose,'pose_odom':latest['odom'],'simulation_seconds':sample()['simulation_seconds'],
            'known_area_m2':float(np.count_nonzero(data>=0)*m.info.resolution**2),
            'range_max_m':float(raw.range_max),'directions':direction_summary(rays),
            'rays':rays,'map':view,'distance_rule':'half of lidar range in selected direction; no footprint margin'}
    try:
        ready_until=min(deadline,time.monotonic()+15)
        while not all(k in latest for k in ('odom','scan','map')) or not buffer.can_transform('map','vehicle/base_link',rclpy.time.Time()):
            spin()
            if time.monotonic()>ready_until:raise RuntimeError('initial_feedback_unavailable')
        if node.count_publishers('/model/vehicle/cmd_vel_safe')!=1:
            raise RuntimeError('another_motion_publisher_active')
        report['before']=sample()
        if request['action']=='scan':report['turned_rad']=turn(2*math.pi)
        elif request['action']=='drive':
            pose=map_pose();raw=latest['scan']
            rays=scan_rays(raw.ranges,raw.angle_min,raw.angle_increment,raw.range_min,raw.range_max,pose[2])
            goal=half_range_goal(rays,float(request['heading_map_rad']),pose);report['goal']=goal
            target_odom=wrap(latest['odom'][2]+wrap(goal['heading_map_rad']-pose[2]))
            report['turned_rad']=turn(wrap(target_odom-latest['odom'][2]))
            phase='driving';anchor=list(latest['odom']);travelled=0.
            while travelled<goal['distance_m']-.04:
                spin();p=latest['odom'];travelled=(p[0]-anchor[0])*math.cos(target_odom)+(p[1]-anchor[1])*math.sin(target_odom)
                remaining=goal['distance_m']-travelled
                if remaining<=.04:break
                command(min(.6,max(.08,remaining*1.2)),max(-.8,min(.8,2*wrap(target_odom-p[2]))))
                time.sleep(.02)
            report['forward_odom_m']=travelled
        elif request['action'] not in ('observe','stop'):raise ValueError('unknown_action')
        settle();report['status']='succeeded'
    except Exception as error:report.update(status='failed',reason=str(error))
    finally:
        settle();phase='finished';report.update(after=sample(),distance_odom_m=distance,wall_elapsed_s=time.monotonic()-started)
        if all(k in latest for k in ('odom','scan','map')):
            try:report['observation']=snapshot()
            except Exception as error:report['snapshot_error']=type(error).__name__
        output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps({'result':str(output),'status':report['status']}),flush=True)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
