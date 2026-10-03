#!/usr/bin/env python3
"""Send one map-frame Nav2 goal; record its plan, execution and safe stop.

No waypoints or velocity control are used. Ground truth is observation only.
"""
import argparse
import json
import math
import signal
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from tf2_ros import Buffer, TransformListener
from gz.transport import Node as GzNode
from gz.msgs.pose_v_pb2 import Pose_V


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--x',type=float,required=True);ap.add_argument('--y',type=float,required=True)
    ap.add_argument('--yaw',type=float,default=0.0);ap.add_argument('--output',required=True)
    ap.add_argument('--plan-only',action='store_true');ap.add_argument('--expect-failure',action='store_true')
    ap.add_argument('--timeout',type=float,default=900)
    ap.add_argument('--expected-error-code',type=int,default=206)
    args=ap.parse_args();output=Path(args.output)
    if output.exists():raise RuntimeError('Choose a new report name')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,signal.default_int_handler);signal.signal(signal.SIGTERM,signal.default_int_handler)
    node=rclpy.create_node('nav2_goal_check',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    buffer=Buffer();listener=TransformListener(buffer,node);latest={};trace=[];plans=[];handle=None
    result={'passed':False,'goal':[args.x,args.y,args.yaw]};started=time.monotonic()
    def odom(m):latest.update(velocity=[m.twist.twist.linear.x,m.twist.twist.angular.z],odom_at=time.monotonic())
    def scan(m):
        hits=[v for v in m.ranges if math.isfinite(v) and m.range_min<=v<=m.range_max]
        latest.update(nearest=min(hits,default=0.),scan_at=time.monotonic())
    def truth(m):
        p=next((p for p in m.pose if p.name=='vehicle'),None)
        if p is not None:
            a=yaw(p.orientation)
            latest['truth']=[p.position.x+.5542825*math.cos(a),p.position.y+.5542825*math.sin(a),a]
    def plan(m):
        if len(plans)<200:plans.append([[p.pose.position.x,p.pose.position.y] for p in m.poses])
    gz=GzNode();gz.subscribe(Pose_V,'/world/demo/dynamic_pose/info',truth)
    subs=[node.create_subscription(Odometry,'/model/vehicle/odometry',odom,10),
          node.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data),
          node.create_subscription(RosPath,'/plan',plan,10)]
    nav=ActionClient(node,NavigateToPose,'/navigate_to_pose')
    planner=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
    def spin():rclpy.spin_once(node,timeout_sec=.05)
    def wait_future(f,timeout):
        until=time.monotonic()+timeout
        while not f.done():
            if time.monotonic()>until:raise RuntimeError('Action/service response timed out')
            spin()
        return f.result()
    def sample():
        t=buffer.lookup_transform('map','vehicle/base_link',rclpy.time.Time()).transform
        return {'wall_elapsed':time.monotonic()-started,'map_pose':[t.translation.x,t.translation.y,yaw(t.rotation)],
            'truth':latest.get('truth'),'nearest':latest.get('nearest'),'velocity':latest.get('velocity')}
    def feedback(msg):
        f=msg.feedback
        latest['feedback']={'distance_remaining':f.distance_remaining,'recoveries':f.number_of_recoveries}
    try:
        if not planner.wait_for_server(timeout_sec=30):raise RuntimeError('Planner unavailable')
        until=time.monotonic()+30
        while not all(k in latest for k in ['odom_at','scan_at','truth']) or not buffer.can_transform('map','vehicle/base_link',rclpy.time.Time()):
            if time.monotonic()>until:raise RuntimeError('Missing robot feedback')
            spin()
        result['start']=sample()
        pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=node.get_clock().now().to_msg()
        pose.pose.position.x=args.x;pose.pose.position.y=args.y
        pose.pose.orientation.z=math.sin(args.yaw/2);pose.pose.orientation.w=math.cos(args.yaw/2)
        pg=ComputePathToPose.Goal();pg.goal=pose;pg.planner_id='GridBased';pg.use_start=False
        ph=wait_future(planner.send_goal_async(pg),10)
        if not ph.accepted:raise RuntimeError('Planner action rejected')
        pr=wait_future(ph.get_result_async(),30)
        result['preflight']={'status':pr.status,'error_code':pr.result.error_code,'error_msg':pr.result.error_msg,
            'path':[[p.pose.position.x,p.pose.position.y] for p in pr.result.path.poses]}
        print(json.dumps({'preflight_status':pr.status,'path_points':len(pr.result.path.poses),'error':pr.result.error_msg}),flush=True)
        if args.plan_only:
            result['passed']=pr.status==4
            if not result['passed']:raise RuntimeError('Preflight planning failed')
            return
        if not args.expect_failure and pr.status!=4:raise RuntimeError('Preflight planning failed')
        if not nav.wait_for_server(timeout_sec=10):raise RuntimeError('Navigator unavailable')
        goal=NavigateToPose.Goal();goal.pose=pose
        handle=wait_future(nav.send_goal_async(goal,feedback_callback=feedback),10)
        if not handle.accepted:raise RuntimeError('Navigation action rejected')
        future=handle.get_result_async();until=time.monotonic()+args.timeout;last_log=last_trace=0
        min_scan=math.inf;max_vel=[0.,0.]
        while not future.done():
            spin();now=time.monotonic()
            if now>until:raise RuntimeError('Navigation execution deadline')
            if any(now-latest[k]>2 for k in ['odom_at','scan_at']):raise RuntimeError('Stale feedback during navigation')
            min_scan=min(min_scan,latest['nearest'])
            max_vel=[max(a,abs(b)) for a,b in zip(max_vel,latest['velocity'])]
            if now-last_trace>.5:
                trace.append(sample());last_trace=now
            if now-last_log>10:
                print(json.dumps({'progress':sample(),**latest.get('feedback',{})}),flush=True);last_log=now
        response=future.result();handle=None
        result.update(action_status=response.status,error_code=response.result.error_code,error_msg=response.result.error_msg,
                      minimum_scan_m=min_scan,max_abs_velocity=max_vel,feedback=latest.get('feedback'))
        until=time.monotonic()+3
        while time.monotonic()<until:spin()
        result['end']=sample()
        result['goal_position_error_m']=math.dist(result['end']['map_pose'][:2],[args.x,args.y])
        result['goal_heading_error_rad']=abs(math.atan2(math.sin(result['end']['map_pose'][2]-args.yaw),math.cos(result['end']['map_pose'][2]-args.yaw)))
        result['stopped']=max(abs(v) for v in latest['velocity'])<.02
        result['rejection_displacement_m']=math.dist(result['start']['truth'][:2],result['end']['truth'][:2])
        result['passed']=((response.status==6 and response.result.error_code==args.expected_error_code
            and pr.status==6 and pr.result.error_code==args.expected_error_code
            and result['rejection_displacement_m']<.05) if args.expect_failure else
            response.status==4 and result['goal_position_error_m']<.20 and result['goal_heading_error_rad']<.15) and result['stopped']
        if not result['passed']:raise RuntimeError('Navigation outcome did not meet expected criteria')
    except BaseException as e:
        result['error']=str(e) or type(e).__name__
        raise
    finally:
        if handle is not None:
            try:
                cancellation=wait_future(handle.cancel_goal_async(),10)
                result['cancel_accepted']=bool(cancellation.goals_canceling)
                wait_future(handle.get_result_async(),10)
            except Exception as e:result['cancel_error']=str(e)
        result['elapsed_wall_seconds']=time.monotonic()-started
        output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(result,indent=2)+'\n')
        output.with_suffix('.trace.json').write_text(json.dumps({'trace':trace,'plans':plans})+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='preflight'}),flush=True)
        gz.unsubscribe('/world/demo/dynamic_pose/info');del gz
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
