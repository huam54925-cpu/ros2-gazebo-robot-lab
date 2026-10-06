#!/usr/bin/env python3
"""Bounded map-only Frontier baseline over Nav2 actions; no velocity publisher."""
import argparse
from dataclasses import asdict
import fcntl
import hashlib
import json
import math
import signal
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose, ComputePathToPose, Spin
from nav2_msgs.srv import ManageLifecycleNodes
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener
from frontier import FrontierMap, Settings
from aggressive import AggressiveMap, settings as aggressive_settings, utility
from path_safety import PathSafety, remaining_path
from safety_contract import STOP_RADIUS_M, SCAN_TIMEOUT_S, SCAN_FRAME, BASE_FRAME, scan_state
from map_identity import map_version
from safety_profile import FOOTPRINT_MODE, PROFILE


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


class MissionEnd(Exception):
    pass


class Explorer:
    def __init__(self,args):
        self.args=args;self.output=Path(args.output)
        if self.output.exists():raise RuntimeError('Output already exists; choose a new name')
        self.output.parent.mkdir(parents=True,exist_ok=True)
        owner_path=Path(__file__).resolve().parents[2]/'log'/'.navigation-owner.lock'
        owner_path.parent.mkdir(parents=True,exist_ok=True)
        self.motion_lock=owner_path.open('w')
        fcntl.flock(self.motion_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.lock=open(self.output.parent/'.frontier.lock','w')
        fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.node=rclpy.create_node('frontier_explorer',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.buffer=Buffer(node=self.node);self.listener=TransformListener(self.buffer,self.node)
        self.latest={};self.traces=[];self.history=[];self.goals=[];self.events=[];self.handle=None;self.action_result=None
        self.distance=0.;self.last_odom=None;self.minimum_scan=math.inf
        self.clearance_model=None;self.active_candidate=None;self.execution_path=None
        self.path_revision=0
        self.config=aggressive_settings() if args.policy=="aggressive" else Settings();self.started=time.monotonic();self.last_trace=self.last_progress=0
        self.report={'status':'running','stop_reason':None,'policy':args.policy,'configuration':asdict(self.config),
            'limits':{'wall_seconds':args.wall_budget,'maximum_goals':args.max_goals,'goal_wall_timeout':args.goal_timeout},
            'safety_profile':PROFILE,'ground_truth_used':False,'coverage':None,'coverage_note':'No fixed explorable-area denominator; only known-cell area is reported.'}
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subs=[self.node.create_subscription(OccupancyGrid,'/map',self.map_cb,qos),
            self.node.create_subscription(Odometry,'/model/vehicle/odometry',self.odom_cb,10),
            self.node.create_subscription(LaserScan,'/scan',self.scan_cb,qos_profile_sensor_data),
            self.node.create_subscription(NavPath,'/plan',self.path_cb,1),
            self.node.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',lambda m:self.latest.update(status=m),qos)]
        self.nav=ActionClient(self.node,NavigateToPose,'/navigate_to_pose')
        self.planner=ActionClient(self.node,ComputePathToPose,'/compute_path_to_pose')
        self.spin_client=ActionClient(self.node,Spin,'/spin')
        self.manager=self.node.create_client(ManageLifecycleNodes,'/lifecycle_manager_navigation/manage_nodes')
        self.event_file=self.output.with_suffix('.events.jsonl').open('x')

    def map_cb(self,m):
        if m.header.frame_id!='map':raise RuntimeError('unexpected_map_frame')
        data=np.asarray(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
        digest=map_version(m)
        self.latest.update(map=m,data=data,map_at=time.monotonic(),map_version=digest)

    def odom_cb(self,m):
        p=m.pose.pose.position;xy=(p.x,p.y)
        if self.last_odom is not None:
            d=math.dist(xy,self.last_odom)
            if d>1:raise RuntimeError('odometry_discontinuity')
            self.distance+=d
        self.last_odom=xy
        self.latest.update(velocity=[m.twist.twist.linear.x,m.twist.twist.angular.z],odom_at=time.monotonic())

    def scan_cb(self,m):
        nearest,valid=scan_state(m)
        self.latest.update(nearest=nearest,scan_at=time.monotonic(),scan_valid=valid,
                           scan_range_max_m=m.range_max)
        self.minimum_scan=min(self.minimum_scan,nearest)

    def path_cb(self,m):
        if self.active_candidate is None:return
        # During navigation no candidate ComputePath request is issued. Only
        # accept a current plan for this goal, never another goal's latched path.
        if rclpy.time.Time.from_msg(m.header.stamp).nanoseconds < self.execution_started_ns:return
        self.execution_path=([[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in m.poses]
                             if m.header.frame_id=='map' else [])
        self.path_revision+=1

    def evaluate_path(self,path,candidate):
        version=self.latest['map_version']
        if self.clearance_model is None or self.clearance_model.map_version!=version:
            m=self.latest['map'];o=m.info.origin
            self.clearance_model=PathSafety(self.latest['data'],m.info.resolution,
                (o.position.x,o.position.y,yaw(o.orientation)),version)
        try:t=self.buffer.lookup_transform(BASE_FRAME,SCAN_FRAME,rclpy.time.Time())
        except Exception as e:raise RuntimeError('lidar_transform_unavailable') from e
        offset=(t.transform.translation.x,t.transform.translation.y)
        return self.clearance_model.evaluate(path,self.pose(),
            [candidate['x'],candidate['y'],candidate['yaw']],offset,
            required_clearance_m=getattr(self,'required_path_clearance_m',None))

    def spin_once(self):rclpy.spin_once(self.node,timeout_sec=.05)

    def event(self,event_type,**values):
        event={'event':event_type,'wall_elapsed':time.monotonic()-self.started,**values}
        self.events.append(event);self.event_file.write(json.dumps(event,allow_nan=False)+'\n');self.event_file.flush()
        print(json.dumps(event,allow_nan=False),flush=True)

    def wait(self,future,seconds):
        until=time.monotonic()+seconds
        while not future.done():
            if time.monotonic()>until:raise RuntimeError('action_response_timeout')
            self.spin_once()
        return future.result()

    def pose(self):
        try:t=self.buffer.lookup_transform('map','vehicle/base_link',rclpy.time.Time())
        except Exception as e:raise RuntimeError('localization_unavailable') from e
        age=(self.node.get_clock().now()-rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds*1e-9
        if age>2:raise RuntimeError('localization_stale')
        return [t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)]

    def health(self):
        now=time.monotonic()
        from robot_contract import CONTRACT
        deadlines=[('odom_at',CONTRACT['feedback_wall_timeout_s']),
                   ('scan_at',CONTRACT['feedback_wall_timeout_s']),('map_at',20.)]
        if any(now-self.latest.get(key,-math.inf)>timeout for key,timeout in deadlines):
            # Synchronous candidate/map geometry can outlast the scan period.
            # Drain already queued feedback before judging its age; do not wait
            # for a missing sensor or relax the independent guard's timeout.
            for _ in range(32):rclpy.spin_once(self.node,timeout_sec=0.)
            now=time.monotonic()
        for key,timeout in deadlines:
            if now-self.latest.get(key,-math.inf)>timeout:raise RuntimeError(key+'_stale')
        if not self.latest.get('scan_valid'):raise RuntimeError('scan_invalid')
        self.pose()

    def sample(self):
        m=self.latest['map'];d=self.latest['data']
        return {'wall_elapsed':time.monotonic()-self.started,'simulation_seconds':self.node.get_clock().now().nanoseconds*1e-9,
            'pose':self.pose(),'known_cells':int((d>=0).sum()),'known_area_m2':float((d>=0).sum()*m.info.resolution**2),
            'map_version':self.latest['map_version'],'map_width':m.info.width,'map_height':m.info.height,
            'path_length_odom_m':self.distance,'nearest_scan_m':self.latest['nearest'] if math.isfinite(self.latest['nearest']) else None,
            'velocity':self.latest['velocity'][:]}

    def model(self):
        m=self.latest['map'];o=m.info.origin
        origin=(o.position.x,o.position.y,yaw(o.orientation))
        if self.args.policy=="aggressive":
            try:t=self.buffer.lookup_transform(BASE_FRAME,SCAN_FRAME,rclpy.time.Time())
            except Exception as error:raise RuntimeError('lidar_transform_unavailable') from error
            return AggressiveMap(self.latest['data'],m.info.resolution,origin,self.config,
                                 (t.transform.translation.x,t.transform.translation.y),
                                 self.latest.get('scan_range_max_m'))
        return FrontierMap(self.latest['data'],m.info.resolution,origin,self.config)

    def save_map(self,label):
        m=self.latest['map'];o=m.info.origin
        np.savez_compressed(self.output.with_suffix('.'+label+'.npz'),data=self.latest['data'],resolution=m.info.resolution,
            origin=[o.position.x,o.position.y,yaw(o.orientation)])

    def stop_confirmed(self,timeout=8.):
        until=time.monotonic()+timeout;since=None
        while time.monotonic()<until:
            self.spin_once();now=time.monotonic()
            if now-self.latest.get('odom_at',-math.inf)<1.5 and max(map(abs,self.latest.get('velocity',[1,1])))<.02:
                since=since or now
                if now-since>.7:return True
            else:since=None
        return False

    def cancel_active(self):
        if self.handle is None:return self.stop_confirmed()
        try:
            response=self.wait(self.handle.cancel_goal_async(),8)
            outcome=self.wait(self.action_result,10)
            self.event('canceled_action',accepted=bool(response.goals_canceling),status=outcome.status)
            self.handle=None;self.action_result=None
            if self.stop_confirmed():return True
        except Exception as e:self.event('cancel_error',message=str(e))
        # Lifecycle pause removes all Nav2 command sources; the independent guard remains.
        if self.manager.wait_for_service(timeout_sec=2):
            req=ManageLifecycleNodes.Request();req.command=ManageLifecycleNodes.Request.PAUSE
            try:self.event('navigation_paused',success=self.wait(self.manager.call_async(req),10).success)
            except Exception as e:self.event('pause_error',message=str(e))
        self.handle=None;self.action_result=None
        return self.stop_confirmed()

    def observe(self,seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:self.spin_once();self.health()

    def execute(self,client,goal,timeout,kind):
        if self.handle is not None:raise RuntimeError('concurrent_action_disallowed')
        if not FOOTPRINT_MODE and self.latest['nearest']<STOP_RADIUS_M:
            if not self.stop_confirmed():raise RuntimeError('stop_unconfirmed')
            return {'status':5,'error_msg':'guard_stop'}
        self.handle=self.wait(client.send_goal_async(goal),10)
        if not self.handle.accepted:self.handle=None;return {'status':0,'error_code':1,'error_msg':'action_rejected'}
        self.action_result=self.handle.get_result_async();deadline=time.monotonic()+timeout
        progress_anchor=self.pose();progress_at=time.monotonic()
        checked_key=None;checked_at=-math.inf
        while not self.action_result.done():
            self.spin_once();self.health();now=time.monotonic()
            if not FOOTPRINT_MODE and self.latest['nearest']<STOP_RADIUS_M:
                if not self.cancel_active():raise RuntimeError('stop_unconfirmed')
                return {'status':5,'error_msg':'guard_stop'}
            if kind=='navigation':
                key=(self.latest['map_version'],self.path_revision)
                if key!=checked_key or now-checked_at>1.0:
                    checked_path=remaining_path(self.execution_path,self.pose())
                    check=self.evaluate_path(checked_path,self.active_candidate)
                    checked_key=key;checked_at=now
                    if not check['safe']:
                        self.event('execution_path_invalidated',candidate=self.active_candidate,clearance=check)
                        # Freeze the exact failing map/path before cancellation spins
                        # callbacks. Write only after confirming stop; disk I/O must
                        # never delay cancellation or replace its outcome.
                        m=self.latest['map'];o=m.info.origin
                        failure_grid=self.latest['data'].copy()
                        failure_resolution=m.info.resolution
                        failure_origin=[o.position.x,o.position.y,yaw(o.orientation)]
                        failure={'map_version':self.latest['map_version'],'path':checked_path,
                                 'start':self.pose(),'candidate':dict(self.active_candidate),
                                 'clearance':check,'map_stamp_sim_s':m.header.stamp.sec+m.header.stamp.nanosec/1e9}
                        if not self.cancel_active():raise RuntimeError('stop_unconfirmed')
                        try:
                            np.savez_compressed(self.output.with_suffix('.invalidated.npz'),data=failure_grid,
                                                resolution=failure_resolution,origin=failure_origin)
                            self.output.with_suffix('.invalidated.json').write_text(json.dumps(failure,indent=2))
                        except Exception as error:
                            self.event('failure_snapshot_write_failed',error_type=type(error).__name__)
                        return {'status':5,'error_msg':check['reason'],'clearance':check}
            if now-self.started>=self.args.wall_budget:
                if not self.cancel_active():raise RuntimeError('stop_unconfirmed')
                return {'status':5,'error_msg':'budget_exhausted'}
            if now>deadline:
                if not self.cancel_active():raise RuntimeError('stop_unconfirmed')
                return {'status':5,'error_msg':'goal_timeout'}
            p=self.pose()
            if math.dist(p[:2],progress_anchor[:2])>.08 or abs(math.atan2(math.sin(p[2]-progress_anchor[2]),math.cos(p[2]-progress_anchor[2])))>.10:
                progress_anchor=p;progress_at=now
            if kind!='short_motion' and now-progress_at>40:
                if not self.cancel_active():raise RuntimeError('stop_unconfirmed')
                return {'status':5,'error_msg':'no_motion_progress'}
            if now-self.last_trace>1:
                self.traces.append(self.sample());self.last_trace=now
            if now-self.last_progress>15:
                self.event('progress',kind=kind,**self.sample());self.last_progress=now
        response=self.action_result.result();self.handle=None;self.action_result=None
        if not self.stop_confirmed():raise RuntimeError('stop_unconfirmed')
        return {'status':response.status,'error_code':response.result.error_code,'error_msg':response.result.error_msg}

    def plan(self,candidate):
        g=ComputePathToPose.Goal();g.goal=self.goal_pose(candidate);g.planner_id='GridBased';g.use_start=False
        h=self.wait(self.planner.send_goal_async(g),10)
        if not h.accepted:return None,{'status':0,'error_code':1}
        future=h.get_result_async()
        try:r=self.wait(future,15)
        except Exception:
            self.wait(h.cancel_goal_async(),5);raise
        path=[[p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)] for p in r.result.path.poses]
        if r.status!=4:
            no_path_codes={getattr(ComputePathToPose.Result,name,None) for name in
                           ('NO_VALID_PATH','START_OCCUPIED','GOAL_OCCUPIED')}-{None}
            return None,{'status':r.status,'error_code':r.result.error_code,'message':r.result.error_msg,
                         'failure_category':'planner_no_path' if r.result.error_code in no_path_codes else 'planner_system_error'}
        if r.result.path.header.frame_id!='map':return None,{'status':0,'error_code':1,'message':'invalid_path_frame'}
        clearance=self.evaluate_path(path,candidate)
        outcome={'status':4,'error_code':0,'clearance':clearance}
        if not clearance['safe']:
            self.event('planning_geometry_rejected',candidate=candidate,checked_path=path,start_pose=self.pose(),clearance=clearance)
            return None,{**outcome,'path_evidence_file':self.output.with_suffix('.events.jsonl').name}
        return {'length':sum(math.dist(a[:2],b[:2]) for a,b in zip(path,path[1:])),
                'path':path,'clearance':clearance},outcome

    def goal_pose(self,c):
        p=PoseStamped();p.header.frame_id='map';p.header.stamp=self.node.get_clock().now().to_msg()
        p.pose.position.x=c['x'];p.pose.position.y=c['y'];p.pose.orientation.z=math.sin(c['yaw']/2);p.pose.orientation.w=math.cos(c['yaw']/2)
        return p

    def run(self):
        reason='internal_error'
        try:
            for client in [self.planner,self.nav,self.spin_client]:
                if not client.wait_for_server(timeout_sec=30):raise RuntimeError('nav2_unavailable')
            # Action endpoints exist before lifecycle activation; discovery is insufficient.
            for name in ['planner_server','controller_server','behavior_server','bt_navigator']:
                state_client=self.node.create_client(GetState,'/'+name+'/get_state')
                deadline=time.monotonic()+45
                while True:
                    if time.monotonic()>deadline:raise RuntimeError(name+'_not_active')
                    if not state_client.wait_for_service(timeout_sec=1):continue
                    if self.wait(state_client.call_async(GetState.Request()),5).current_state.id==3:break
                    self.spin_once()
                self.node.destroy_client(state_client)
            until=time.monotonic()+30
            while not all(k in self.latest for k in ['map','scan_at','odom_at']) or not self.buffer.can_transform('map','vehicle/base_link',rclpy.time.Time()):
                if time.monotonic()>until:raise RuntimeError('missing_initial_feedback')
                self.spin_once()
            self.observe(2)
            if 'status' in self.latest and any(s.status in (1,2,3) for s in self.latest['status'].status_list):raise RuntimeError('another_navigation_is_active')
            if not self.stop_confirmed():raise RuntimeError('robot_not_stationary')
            self.report['initial']=self.sample();self.save_map('initial');self.event('started',**self.report['initial'])
            if self.args.initial_scan and not self.args.dry_run:
                if self.latest['nearest']<2.7:raise RuntimeError('insufficient_clearance_for_initial_scan')
                g=Spin.Goal();g.target_yaw=2*math.pi;g.time_allowance.sec=90;g.disable_collision_checks=False
                outcome=self.execute(self.spin_client,g,max(1,self.args.wall_budget-(time.monotonic()-self.started)),'initial_scan');self.report['initial_scan']=outcome
                self.event('initial_scan_result',**outcome)
                if outcome.get('error_msg')=='budget_exhausted':raise MissionEnd('budget_exhausted')
                if outcome['status']!=4:raise RuntimeError('initial_scan_failed')
                self.observe(8)
            if getattr(self.args,'initialize_only',False):
                raise MissionEnd('initialization_complete')
            while True:
                self.health()
                # Guard blocks rotation as well as translation. Do not dispatch
                # any new goal while it is still latched by the current scan.
                if not FOOTPRINT_MODE and self.latest['nearest']<STOP_RADIUS_M:reason='guard_blocked_no_safe_motion';break
                if time.monotonic()-self.started>=self.args.wall_budget:reason='budget_exhausted';break
                if len(self.goals)>=self.args.max_goals:reason='goal_budget_exhausted';break
                model=self.model();pose=self.pose()
                excluded=[(h['x'],h['y']) for h in self.history]
                if self.args.policy=='aggressive':
                    area=self.sample()['known_area_m2']
                    excluded=[]
                    for h in self.history:
                        nearby=sum(math.hypot(h['x']-g['candidate']['x'],h['y']-g['candidate']['y'])<1.0 for g in self.goals)
                        if (h['outcome']!='visited' or nearby>=2 or len(self.goals)-h.get('goal_index',0)<3
                            or area-h.get('known_area',area)<10.0):
                            excluded.append((h['x'],h['y']))
                candidates,info=model.candidates(pose,excluded)
                self.event('candidates',map_version=self.latest['map_version'],**info,candidates=candidates)
                if self.args.dry_run:reason='dry_run';break
                if not candidates:
                    reason='no_frontiers' if info['frontier_clusters']==0 else 'no_eligible_observation_goals';break
                reachable=[]
                for c in candidates:
                    self.spin_once();self.health()
                    if time.monotonic()-self.started>=self.args.wall_budget:break
                    # Euclidean distance is a lower bound for path length.
                    if self.args.policy=='conservative' and reachable and c['euclidean_distance_m']>min(p['plan']['length'] for p in reachable):break
                    plan,outcome=self.plan(c);self.event('planning_result',candidate=c,**outcome)
                    if plan is not None:reachable.append({'candidate':c,'plan':plan})
                    else:self.history.append({'x':c['x'],'y':c['y'],
                        'outcome':outcome.get('clearance',{}).get('reason','unreachable')})
                if not reachable:
                    reason='budget_exhausted' if time.monotonic()-self.started>=self.args.wall_budget else 'no_reachable_candidates';break
                if self.args.policy=='aggressive':
                    selected=max(reachable,key=lambda p:utility(p['candidate'],p['plan']['length'],pose[2]))
                else:
                    selected=min(reachable,key=lambda p:p['plan']['length'])
                c=selected['candidate']
                # Recheck against the newest map after asynchronous planning.
                # Refresh queued feedback BEFORE validation so a drained map
                # callback cannot invalidate the check immediately before send.
                self.health()
                if not self.model().is_safe(c['x'],c['y'],*([c['yaw']] if self.args.policy=='aggressive' else [])):
                    self.event('goal_invalidated',candidate=c);self.history.append({**c,'outcome':'map_changed'});continue
                # Full route + final rotation must still pass the latest map,
                # even when it changed while other candidates were planned.
                clearance=self.evaluate_path(selected['plan']['path'],c)
                if not clearance['safe']:
                    self.event('path_invalidated_before_dispatch',candidate=c,clearance=clearance)
                    self.history.append({**c,'outcome':clearance['reason']});continue
                if time.monotonic()-self.started>=self.args.wall_budget:reason='budget_exhausted';break
                before=self.sample();entry={'candidate':c,'planned_length_m':selected['plan']['length'],
                    'plan':[p[:2] for p in selected['plan']['path']],
                    'plan_poses':selected['plan']['path'],'clearance':clearance,'before':before}
                self.event('goal_selected',candidate=c,planned_length_m=entry['planned_length_m'],clearance=clearance)
                g=NavigateToPose.Goal();g.pose=self.goal_pose(c)
                self.active_candidate=c;self.execution_path=selected['plan']['path']
                self.execution_started_ns=self.node.get_clock().now().nanoseconds
                result=self.execute(self.nav,g,self.args.goal_timeout,'navigation');entry['result']=result
                self.active_candidate=None;self.execution_path=None
                self.observe(8);entry['after']=self.sample();entry['known_area_gain_m2']=entry['after']['known_area_m2']-before['known_area_m2']
                self.goals.append(entry);self.history.append({**c,'outcome':'visited' if result['status']==4 else 'failed','goal_index':len(self.goals),'known_area':entry['after']['known_area_m2']})
                self.event('goal_result',candidate=c,result=result,known_area_gain_m2=entry['known_area_gain_m2'])
                self.checkpoint()
                if result.get('error_msg')=='budget_exhausted':reason='budget_exhausted';break
            self.report['status']='finished'
        except MissionEnd as e:reason=str(e);self.report['status']='finished'
        except KeyboardInterrupt:reason='operator_stopped';self.report['status']='interrupted'
        except Exception as e:reason=str(e) or type(e).__name__;self.report['status']='error';self.event('error',message=reason)
        finally:
            stopped=self.cancel_active()
            self.report.update(stop_reason=reason,stopped=stopped)
            try:self.report['final']=self.sample();self.save_map('final')
            except Exception as e:self.report['final_sample_error']=str(e)
            self.checkpoint();self.event('finished',status=self.report['status'],stop_reason=reason,stopped=stopped)
            self.event_file.close();self.node.destroy_node();self.lock.close();self.motion_lock.close()
        return self.report['status']!='error' and self.report['stopped']

    def checkpoint(self):
        self.report.update(goals=self.goals,goal_count=len(self.goals),successful_goals=sum(g['result']['status']==4 for g in self.goals),
            failed_goals=sum(g['result']['status']!=4 for g in self.goals),excluded_observation_positions=self.history,
            path_length_odom_m=self.distance,elapsed_wall_seconds=time.monotonic()-self.started,
            minimum_scan_m=self.minimum_scan if math.isfinite(self.minimum_scan) else None)
        temp=self.output.with_suffix('.tmp');temp.write_text(json.dumps(self.report,indent=2,allow_nan=False)+'\n');temp.replace(self.output)
        self.output.with_suffix('.trace.json').write_text(json.dumps(self.traces,allow_nan=False)+'\n')


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--dry-run',action='store_true')
    p.add_argument('--policy',choices=['conservative','aggressive'],default='conservative')
    p.add_argument('--initialize-only',action='store_true',help='Protected initial scan only; no frontier navigation')
    p.add_argument('--initial-scan',action='store_true');p.add_argument('--max-goals',type=int,default=8)
    p.add_argument('--wall-budget',type=float,default=1200);p.add_argument('--goal-timeout',type=float,default=300)
    args=p.parse_args()
    if args.initialize_only:args.initial_scan=True
    if args.max_goals<1 or args.wall_budget<=0 or args.goal_timeout<=0:p.error('budgets must be positive')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,signal.default_int_handler);signal.signal(signal.SIGTERM,signal.default_int_handler)
    try:ok=Explorer(args).run()
    finally:rclpy.shutdown()
    if not ok:raise SystemExit(1)


if __name__=='__main__':main()
