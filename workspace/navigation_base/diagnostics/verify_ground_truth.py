#!/usr/bin/env python3
"""Compare wheel odometry against named Gazebo poses during a bounded motion test."""
import argparse
import json
import math
import signal
import threading
import time
from pathlib import Path
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from gz.transport import Node as GzNode
from gz.msgs.pose_v_pb2 import Pose_V


def wrap(x): return math.atan2(math.sin(x), math.cos(x))
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def rotate(x,y,a): return [math.cos(a)*x-math.sin(a)*y, math.sin(a)*x+math.cos(a)*y]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True)
    parser.add_argument('--expected-frame',default='vehicle/base_link')
    parser.add_argument('--require-tf-consistency',action='store_true')
    args=parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,signal.default_int_handler)
    signal.signal(signal.SIGTERM,signal.default_int_handler)
    n=rclpy.create_node('ground_truth_motion_check')
    buffer=Buffer();listener=TransformListener(buffer,n)
    lock=threading.Lock();data={};trace=[]
    def gz_cb(m):
        model=next((p for p in m.pose if p.name=='vehicle'),None)
        if model is None:return
        points={}
        for name in ['chassis','left_wheel','right_wheel']:
            p=next((p for p in m.pose if p.name==name),None)
            if p is not None:points[name]=[p.position.x,p.position.y]
        with lock:
            data['truth']={'stamp':m.header.stamp.sec+m.header.stamp.nsec*1e-9,
                'model':[model.position.x,model.position.y,yaw(model.orientation)],'links':points}
            data['truth_at']=time.monotonic()
    gz=GzNode();gz.subscribe(Pose_V,'/world/demo/dynamic_pose/info',gz_cb)
    def odom(m):
        p=m.pose.pose
        data['odom']={'stamp':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
            'pose':[p.position.x,p.position.y,yaw(p.orientation)],
            'frame':m.child_frame_id,'velocity':[m.twist.twist.linear.x,m.twist.twist.angular.z]}
        data['odom_at']=time.monotonic()
    def scan(m):
        hits=[v for v in m.ranges if math.isfinite(v) and m.range_min<=v<=m.range_max]
        data['nearest']=min(hits,default=math.inf);data['scan_at']=time.monotonic()
    subs=[n.create_subscription(Odometry,'/model/vehicle/odometry',odom,10),
          n.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data)]
    pub=n.create_publisher(Twist,'/model/vehicle/cmd_vel',1)
    result={'completed':False,'expected_frame':args.expected_frame,'stages':[]}
    def spin():rclpy.spin_once(n,timeout_sec=.05)
    def sample():
        with lock:
            g=data['truth'];model=g['model'];links=g['links']
            offsets={'chassis':links['chassis'],
                     'axle':[(links['left_wheel'][0]+links['right_wheel'][0])/2,
                             (links['left_wheel'][1]+links['right_wheel'][1])/2]}
            world={}
            for key,p in offsets.items():
                d=rotate(*p,model[2]);world[key]=[model[0]+d[0],model[1]+d[1],model[2]]
            result_sample={'odom':dict(data['odom']),'truth_stamp':g['stamp'],'world':world,'nearest':data['nearest']}
            try:
                t=buffer.lookup_transform('vehicle/odom','vehicle/chassis',rclpy.time.Time())
                result_sample['tf_chassis']=[t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)]
            except Exception:
                result_sample['tf_chassis']=None
            return result_sample
    def settle():
        until=time.monotonic()+2.5
        while time.monotonic()<until:pub.publish(Twist());spin();time.sleep(.03)
        return sample()
    try:
        deadline=time.monotonic()+30
        while not all(k in data for k in ['truth','odom','nearest']):
            if time.monotonic()>deadline:raise RuntimeError('Missing truth, odom or laser')
            spin()
        assert data['odom']['frame']==args.expected_frame,data['odom']
        if pub.get_subscription_count()!=1:raise RuntimeError('Expected exactly one velocity guard subscriber')
        start=settle();result['start']=start
        if start['nearest']<2.5:raise RuntimeError('Insufficient initial clearance')
        # Only a short straight segment, rotations and return; no world-pose feedback drives the robot.
        for label,kind,target in [('forward','linear',.6),('left_90','angular',math.pi/2),
                                  ('heading_return','angular',-math.pi/2),('reverse_return','linear',-.6)]:
            before=settle();initial=before['odom']['pose'];deadline=time.monotonic()+90
            while True:
                spin();now=time.monotonic()
                if now>deadline:raise RuntimeError('Motion deadline')
                if any(now-data[k]>1.5 for k in ['truth_at','odom_at','scan_at']):raise RuntimeError('Stale feedback')
                if data['nearest']<2.0:raise RuntimeError('Obstacle too near; test stopped')
                p=data['odom']['pose']
                progress=(rotate(p[0]-initial[0],p[1]-initial[1],-initial[2])[0]
                          if kind=='linear' else wrap(p[2]-initial[2]))
                error=target-progress
                if abs(error)<(.015 if kind=='linear' else .012):break
                c=Twist();speed=math.copysign(min(.15 if kind=='linear' else .2,max(.035,.7*abs(error))),error)
                if kind=='linear':c.linear.x=speed
                else:c.angular.z=speed
                pub.publish(c);trace.append({'stage':label,**sample()});time.sleep(.04)
            after=settle();entry={'name':label,'before':before,'after':after}
            od0,od1=before['odom']['pose'],after['odom']['pose']
            od_delta=rotate(od1[0]-od0[0],od1[1]-od0[1],-od0[2])
            entry['odom_delta_m']=od_delta;entry['odom_yaw_delta_deg']=math.degrees(wrap(od1[2]-od0[2]))
            entry['errors']={}
            for key in ['axle','chassis']:
                w0,w1=before['world'][key],after['world'][key]
                delta=rotate(w1[0]-w0[0],w1[1]-w0[1],-w0[2])
                entry['errors'][key]={'truth_delta_m':delta,
                    'position_error_m':math.dist(delta,od_delta),
                    'truth_yaw_delta_deg':math.degrees(wrap(w1[2]-w0[2])),
                    'yaw_error_deg':math.degrees(wrap((w1[2]-w0[2])-(od1[2]-od0[2])))}
            if before['tf_chassis'] is not None and after['tf_chassis'] is not None:
                t0,t1=before['tf_chassis'],after['tf_chassis']
                td=rotate(t1[0]-t0[0],t1[1]-t0[1],-t0[2])
                entry['tf_chassis_position_error_m']=math.dist(td,entry['errors']['chassis']['truth_delta_m'])
            result['stages'].append(entry)
            print(json.dumps({'stage':label,'errors':entry['errors']}),flush=True)
        result['end']=settle();result['completed']=True
        result['tf_consistent']=all(e.get('tf_chassis_position_error_m',math.inf)<.03
            and abs(e['errors']['axle']['yaw_error_deg'])<2 for e in result['stages'])
        if args.require_tf_consistency and not result['tf_consistent']:
            raise RuntimeError('TF vs ground truth exceeded 3 cm / 2 degree stage limits')
    except BaseException as e:
        result['error']=str(e) or type(e).__name__
        raise
    finally:
        for _ in range(15):pub.publish(Twist());time.sleep(.05)
        path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
        result['trace_samples']=len(trace)
        path.write_text(json.dumps(result,indent=2)+'\n')
        path.with_suffix('.trace.json').write_text(json.dumps(trace)+'\n')
        gz.unsubscribe('/world/demo/dynamic_pose/info')
        del gz
        n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
