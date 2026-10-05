"""Offline Nav2 ComputePath comparison. Run in a --network none container.

Only a static map, TF and planner exist. No controller, NavigateToPose client,
cmd_vel publisher, task ledger mutation, or live navigation lifecycle request.
"""
import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import numpy as np
import yaml

WORKSPACE=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(WORKSPACE),str(WORKSPACE/'navigation_base/exploration')]
from observation import body_safe,navigation_footprint,navigation_prediction,action_time_estimate,body_sweep
from path_safety import PathSafety,angle_delta,SAMPLE_STEP_M
from regions import transit_candidates,region_id
from exploration_support.grid import Grid,sensor_pose
from robot_skills.compare_viewpoints import visible_unknown_boundary


def main(input_path,output,map_path):
    # Reject host-network invocation. /sys/class/net has only loopback in the
    # intended Docker --network none environment. Still has no motion API.
    if set(os.listdir('/sys/class/net'))!={'lo'}:
        raise RuntimeError('requires_isolated_network_none_container')
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import TransformStamped
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    from lifecycle_msgs.srv import ChangeState
    from nav2_msgs.action import ComputePathToPose
    comparison=json.loads(input_path.read_text());start=comparison['robot_pose'];offset=comparison['base_to_lidar_tf']
    with np.load(map_path) as z:
        data=z['data'].copy();resolution=float(z['resolution']);origin=tuple(z['origin'])
    grid=Grid(data,resolution,origin)
    current=visible_unknown_boundary(grid,sensor_pose(tuple(start),tuple(offset)),comparison['range_m'],180).first_unknown_cells
    cell_radius=math.ceil(.20/resolution)
    near={(r+dy,c+dx) for r,c in current for dy in range(-cell_radius,cell_radius+1)
          for dx in range(-cell_radius,cell_radius+1) if math.hypot(dy,dx)*resolution<=.20+1e-8}
    def gain(candidate):
        p=(candidate['x'],candidate['y'],candidate['yaw'])
        view=visible_unknown_boundary(grid,sensor_pose(p,tuple(offset)),comparison['range_m'],180)
        novel=view.first_unknown_cells-current;robust=view.first_unknown_cells-near
        return {'visible_boundary_cells':len(view.first_unknown_cells),'incremental_boundary_cells':len(novel),
                'robust_incremental_boundary_cells':len(robust),'novelty_ratio':len(novel)/len(view.first_unknown_cells) if view.first_unknown_cells else None,
                'robust_novel_boundary_world_xy':[grid.world(r,c) for r,c in sorted(robust)],
                'actual_map_gain_m2':None}
    safety=PathSafety(data,resolution,origin,comparison['source_map_sha256'])
    config=yaml.safe_load((WORKSPACE/'navigation_base/navigation/nav2.yaml').read_text())
    readonly={k:deepcopy(config[k]) for k in ('/**','planner_server','global_costmap')}
    readonly['/**']['ros__parameters']['use_sim_time']=False
    cp=readonly['global_costmap']['global_costmap']['ros__parameters']
    cp['use_sim_time']=False;cp['plugins']=['static_layer','inflation_layer']
    cp['static_layer']['subscribe_to_updates']=False
    report={'scope':'isolated static snapshot Nav2 planning + unchanged 2.15m path clearance + padded full body sweep; not live motion authorization',
            'source':comparison['source_catalog'],'source_map_sha256':comparison['source_map_sha256'],
            'robot_pose':start,'base_to_lidar_tf':offset,'configuration':readonly,
            'difference_from_live':'live scan obstacle layer unavailable in recorded snapshot; static occupancy and original inflation retained',
            'motion_commands_sent':0,'stop_latch_touched':False,'plans':[],'variants':{}}
    def save():
        output.parent.mkdir(parents=True,exist_ok=True);temp=output.with_suffix('.tmp')
        temp.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n');temp.replace(output)
    rclpy.init();node=rclpy.create_node('readonly_snapshot_planner_client')
    publisher=node.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
    tf=StaticTransformBroadcaster(node)
    message=OccupancyGrid();message.header.frame_id='map';message.header.stamp=node.get_clock().now().to_msg()
    message.info.resolution=resolution;message.info.height=data.shape[0];message.info.width=data.shape[1]
    message.info.origin.position.x=float(origin[0]);message.info.origin.position.y=float(origin[1])
    message.info.origin.orientation.z=math.sin(origin[2]/2);message.info.origin.orientation.w=math.cos(origin[2]/2)
    message.data=data.ravel().astype(int).tolist();publisher.publish(message)
    transform=TransformStamped();transform.header.frame_id='map';transform.child_frame_id='vehicle/base_link';transform.header.stamp=node.get_clock().now().to_msg()
    transform.transform.translation.x=start[0];transform.transform.translation.y=start[1]
    transform.transform.rotation.z=math.sin(start[2]/2);transform.transform.rotation.w=math.cos(start[2]/2);tf.sendTransform(transform)
    process=None;log=None
    def wait(future,timeout):
        end=time.monotonic()+timeout
        while not future.done():
            if time.monotonic()>end:raise TimeoutError('readonly_planner_timeout')
            rclpy.spin_once(node,timeout_sec=.05)
        return future.result()
    def fill(pose,xyz):
        pose.header.frame_id='map';pose.header.stamp=node.get_clock().now().to_msg()
        pose.pose.position.x=float(xyz[0]);pose.pose.position.y=float(xyz[1])
        pose.pose.orientation.z=math.sin(xyz[2]/2);pose.pose.orientation.w=math.cos(xyz[2]/2)
    try:
        with tempfile.TemporaryDirectory() as directory:
            param=Path(directory)/'planner.yaml';param.write_text(yaml.safe_dump(readonly))
            log=output.with_suffix('.planner.log').open('w')
            process=subprocess.Popen(['ros2','run','nav2_planner','planner_server','--ros-args','--params-file',str(param)],stdout=log,stderr=log)
            lifecycle=node.create_client(ChangeState,'/planner_server/change_state')
            if not lifecycle.wait_for_service(timeout_sec=35):raise RuntimeError('planner_lifecycle_missing')
            for state in (1,3):
                request=ChangeState.Request();request.transition.id=state
                if not wait(lifecycle.call_async(request),35).success:raise RuntimeError('planner_lifecycle_failed')
            client=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
            if not client.wait_for_server(timeout_sec=15):raise RuntimeError('planner_action_missing')
            # Allow costmap to receive and inflate the immutable map.
            end=time.monotonic()+2
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
            def plan(candidate):
                goal_pose=[candidate['x'],candidate['y'],candidate['yaw']]
                goal=ComputePathToPose.Goal();goal.planner_id='GridBased';goal.use_start=True
                fill(goal.start,start);fill(goal.goal,goal_pose)
                handle=wait(client.send_goal_async(goal),10)
                if not handle.accepted:return {'planner_success':False,'reason':'planner_goal_rejected'}
                future=handle.get_result_async()
                try:result=wait(future,10)
                except TimeoutError:
                    wait(handle.cancel_goal_async(),5);raise
                if result.status!=4:return {'planner_success':False,'reason':'no_nav2_path','status':result.status,'error_code':result.result.error_code,'error_msg':result.result.error_msg}
                def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                path=[[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in result.result.path.poses]
                if result.result.path.header.frame_id!='map':raise ValueError('invalid_path_frame')
                clearance=safety.evaluate(path,start,goal_pose,offset[:2])
                body=body_sweep(data,resolution,origin,path,start,goal_pose)
                length=sum(math.dist(a[:2],b[:2]) for a,b in zip(path,path[1:]))
                return {'planner_success':True,'path':path,'planned_length_m':length,'clearance':clearance,'body_sweep':body,
                        'geometry_pass':clearance['safe'] and body['safe'],
                        'time_estimate':action_time_estimate(length,navigation_prediction(path,start,goal_pose),resolution)}
            cached={}
            for name,variant in comparison['variants'].items():
                ids=[]
                for c in variant['candidates']:
                    key=tuple(round(c[k],8) for k in ('x','y','yaw'))
                    if key not in cached:
                        candidate={**c,'region_id':region_id((c['x'],c['y']))}
                        row={'plan_id':'PLAN_'+str(len(cached)+1),'candidate':candidate,'visibility':gain(c)}
                        row['full_target']=plan(candidate)
                        row['bounded_action']=None;row['relay_attempts']=[]
                        route=row['full_target']
                        if route.get('geometry_pass') and row['visibility']['robust_incremental_boundary_cells']>0:
                            if route['planned_length_m']<=8:
                                row['bounded_action']={'transit':False,'candidate':candidate,'check':route}
                            else:
                                for relay in transit_candidates(candidate,route['path'],start):
                                    check=plan(relay);row['relay_attempts'].append({'candidate':relay,'check':check})
                                    if check.get('geometry_pass') and check['planned_length_m']<=8:
                                        row['bounded_action']={'transit':True,'candidate':relay,'check':check};break
                        cached[key]=row;report['plans'].append(row);save()
                        print(json.dumps({'plan_id':row['plan_id'],'variant':name,'planner':route.get('planner_success'),
                            'geometry':route.get('geometry_pass'),'length':route.get('planned_length_m'),
                            'visible_new':row['visibility']['robust_incremental_boundary_cells'],'bounded_action':row['bounded_action'] is not None}),flush=True)
                    ids.append(cached[key]['plan_id'])
                rows=[r for r in report['plans'] if r['plan_id'] in ids]
                report['variants'][name]={'planned_candidate_ids':ids,'count':len(ids),
                    'planner_success':sum(r['full_target'].get('planner_success',False) for r in rows),
                    'full_route_geometry_pass':sum(r['full_target'].get('geometry_pass',False) for r in rows),
                    'informative_full_route_pass':sum(bool(r['full_target'].get('geometry_pass') and r['visibility']['robust_incremental_boundary_cells']) for r in rows),
                    'bounded_safe_informative_actions':sum(r['bounded_action'] is not None for r in rows),
                    'direct_goals':sum(r['bounded_action'] is not None and not r['bounded_action']['transit'] for r in rows),
                    'transit_segments':sum(r['bounded_action'] is not None and r['bounded_action']['transit'] for r in rows)}
                save()
            report['complete']=True;save()
    except Exception as error:
        report['error']=type(error).__name__+':'+str(error);save();raise
    finally:
        if process:
            process.terminate()
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        if log:log.close()
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--map',type=Path,required=True)
    a=parser.parse_args();main(a.input,a.output,a.map)
