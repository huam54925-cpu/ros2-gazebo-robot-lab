#!/usr/bin/env python3
"""Supervised fixed route for this indoor world, not an obstacle-avoiding planner."""
import argparse
import json
import math
import signal
import time
from pathlib import Path
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from gz.transport import Node
from gz.msgs.pose_v_pb2 import Pose_V


def wrap(a): return math.atan2(math.sin(a), math.cos(a))
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args=parser.parse_args()
    path=Path(args.output)
    if path.exists(): raise RuntimeError('Use a new output name')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,signal.default_int_handler)
    signal.signal(signal.SIGTERM,signal.default_int_handler)
    node=rclpy.create_node('supervised_indoor_loop')
    buffer=Buffer(); listener=TransformListener(buffer,node)
    latest={}; trace=[]; result={'completed':False,'waypoints':[]}
    def odom(m):
        p=m.pose.pose
        latest.update(pose=[p.position.x,p.position.y,yaw(p.orientation)],
            velocity=[m.twist.twist.linear.x,m.twist.twist.angular.z],odom_at=time.monotonic(),frame=m.child_frame_id)
    def scan(m):
        values=[x for x in m.ranges if math.isfinite(x) and m.range_min<=x<=m.range_max]
        latest.update(nearest=min(values,default=0.),scan_at=time.monotonic())
    def truth(m):
        p=next((p for p in m.pose if p.name=='vehicle'),None)
        if p is not None:
            a=yaw(p.orientation)
            latest.update(truth=[p.position.x+.5542825*math.cos(a),p.position.y+.5542825*math.sin(a),a],truth_at=time.monotonic())
    gz=Node();gz.subscribe(Pose_V,'/world/demo/dynamic_pose/info',truth)
    subs=[node.create_subscription(Odometry,'/model/vehicle/odometry',odom,10),
        node.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data),
        node.create_subscription(OccupancyGrid,'/map',lambda m:latest.update(map=m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
    pub=node.create_publisher(Twist,'/model/vehicle/cmd_vel',1)
    def snapshot():
        t=buffer.lookup_transform('map','vehicle/base_link',rclpy.time.Time()).transform
        return {'wall_time':time.time(),'odom':latest['pose'][:],'truth':latest['truth'][:],
            'map_pose':[t.translation.x,t.translation.y,yaw(t.rotation)],'nearest':latest['nearest'],
            'known_cells':sum(x>=0 for x in latest['map'].data),'velocity':latest['velocity'][:]}
    def spin(): rclpy.spin_once(node,timeout_sec=.05)
    def stop_wait(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until: pub.publish(Twist());spin();time.sleep(.025)
    min_scan=math.inf; last_log=0; started=time.monotonic()
    try:
        until=time.monotonic()+30
        while not all(k in latest for k in ['pose','map','truth','nearest']):
            if time.monotonic()>until: raise RuntimeError('Missing feedback')
            spin()
        stop_wait(2)
        assert latest['frame']=='vehicle/base_link'
        assert math.hypot(*latest['pose'][:2])<.2 and abs(latest['pose'][2])<.1, 'Start at spawn only'
        assert pub.get_subscription_count()==1 and node.count_publishers('/model/vehicle/cmd_vel')==1, 'Another controller or missing guard'
        result['start']=snapshot()
        assert math.dist(latest['truth'][:2],[-7.4457175,-5])<.2,'Unexpected world spawn'
        # World (-7.4457,0), (6.7,0), (6.7,-4.8), (-7.4457,-4.8), spawn.
        route=[(0.,5.),(14.1457175,5.),(14.1457175,.2),(0.,.2),(0.,0.)]
        for gx,gy in route:
            segment=time.monotonic()
            while True:
                spin(); now=time.monotonic()
                if now-started>1500 or now-segment>600:raise RuntimeError('Route deadline')
                if any(now-latest[k]>1.5 for k in ['odom_at','scan_at','truth_at']):raise RuntimeError('Stale feedback')
                min_scan=min(min_scan,latest['nearest'])
                if latest['nearest']<2.0:raise RuntimeError('Clearance below 2.0 m; stopped')
                x,y,a=latest['pose'];distance=math.hypot(gx-x,gy-y)
                if distance<.055:break
                e=wrap(math.atan2(gy-y,gx-x)-a)
                c=Twist();c.angular.z=max(-.22,min(.22,1.4*e))
                if abs(e)<.08:c.linear.x=min(.22,max(.035,.5*distance))
                pub.publish(c)
                if now-last_log>10:
                    sample=snapshot();trace.append(sample)
                    print(json.dumps({'target':[gx,gy],**sample}),flush=True);last_log=now
                time.sleep(.025)
            stop_wait(2)
            s=snapshot();result['waypoints'].append({'target':[gx,gy],**s})
            print(json.dumps({'reached':[gx,gy],**s}),flush=True)
        until=time.monotonic()+90
        while abs(latest['pose'][2])>.025:
            spin()
            if time.monotonic()>until or time.monotonic()-latest['odom_at']>1.5 or time.monotonic()-latest['scan_at']>1.5:raise RuntimeError('Heading restore timeout/stale feedback')
            if latest['nearest']<2:raise RuntimeError('Heading restore clearance')
            c=Twist();c.angular.z=max(-.22,min(.22,-latest['pose'][2]));pub.publish(c)
        stop_wait(12)
        result['end']=snapshot()
        a,b=result['start'],result['end']
        result.update(truth_return_error_m=math.dist(a['truth'][:2],b['truth'][:2]),
            map_return_error_m=math.dist(a['map_pose'][:2],b['map_pose'][:2]),
            truth_heading_return_error_deg=math.degrees(wrap(b['truth'][2]-a['truth'][2])))
        assert max(abs(v) for v in latest['velocity'])<.02,'Not stopped'
        result['completed']=True
    except BaseException as e:
        result['error']=str(e) or type(e).__name__
        raise
    finally:
        stop_wait(1)
        result.update(minimum_observed_scan_m=min_scan,elapsed_wall_seconds=time.monotonic()-started)
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(result,indent=2)+'\n')
        path.with_suffix('.trace.json').write_text(json.dumps(trace,indent=2)+'\n')
        print(json.dumps(result),flush=True)
        gz.unsubscribe('/world/demo/dynamic_pose/info');del gz
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
