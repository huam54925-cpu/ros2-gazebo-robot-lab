"""Nav2 short-motion actions through the same guarded command chain as navigation."""
import math
import time

from navigation_base.robot_contract import CONTRACT
from robot_skills.recovery import POLICY, angle, eligible, straight_retreat, straight_sweep
from robot_skills.recovery import reserve, update_attempt, source_reason, probe_reason


class RecoveryMixin:
    def recovery_feedback(self):
        now=time.monotonic();limits=CONTRACT['profiles']['footprint_075']
        for kind in ('scan','odom'):
            if now-self.latest.get(kind+'_at',-math.inf)>limits[kind+'_timeout_s']:
                raise RuntimeError('short_motion_'+kind+'_stale')
        row=self.odom_trace[-1] if self.odom_trace else None
        sim=self.node.get_clock().now().nanoseconds*1e-9
        if not row or not 0<=sim-row['sim_s']<=limits['odom_timeout_s']:
            raise RuntimeError('short_motion_odom_stamp_stale')
        if row['frame']!=CONTRACT['odom_frame']:raise RuntimeError('recovery_odom_frame_mismatch')
        return row['pose']

    def require_recovery_contract(self):
        self.require_contract()
        from rcl_interfaces.srv import GetParameters
        client=self.node.create_client(GetParameters,'/behavior_server/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=3):raise RuntimeError('behavior_contract_unavailable')
            req=GetParameters.Request()
            req.names=['backup.minimum_speed','drive_on_heading.minimum_speed',
                       'backup.deceleration_limit','drive_on_heading.deceleration_limit',
                       'robot_base_frame','behavior_plugins','local_frame']
            values=self.wait(client.call_async(req),5).values
            if (len(values)!=7 or any(v.type!=3 or abs(v.double_value-POLICY['speed_m_s'])>1e-9 for v in values[:2])
                    or any(v.type!=3 or abs(v.double_value+CONTRACT['linear_deceleration_m_s2'])>1e-9 for v in values[2:4])
                    or values[4].string_value!=CONTRACT['base_frame']
                    or not {'backup','drive_on_heading'}<=set(values[5].string_array_value)
                    or values[6].string_value!=CONTRACT['odom_frame']):
                raise RuntimeError('behavior_contract_mismatch')
        finally:
            self.node.destroy_client(client)

    def short_clearance(self, distance):
        self.recovery_feedback()
        speed=math.copysign(POLICY['speed_m_s'],distance)
        # Reserve the maximum freshness+reaction+full braking travel at the end.
        braking=abs(speed)*(CONTRACT['reaction_time_s']+CONTRACT['profiles']['footprint_075']['scan_timeout_s']
                            +abs(speed)/CONTRACT['linear_deceleration_m_s2'])
        evidence=straight_sweep(self.grid_snapshot(),self.pose(),distance+math.copysign(braking,distance))
        if not evidence['safe']:raise RuntimeError(evidence['reason'])
        from navigation_base.footprint_guard import scan_points,check_sweep
        scan=self.latest.get('raw_scan')
        if scan is None:raise RuntimeError('short_motion_scan_unavailable')
        sim=self.node.get_clock().now().nanoseconds*1e-9
        age=sim-(scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9)
        if not 0<=age<=CONTRACT['profiles']['footprint_075']['scan_timeout_s']:
            raise RuntimeError('short_motion_scan_stamp_stale')
        scan_check=check_sweep(scan_points(scan),(speed,0.),self.latest['velocity'],age)
        if not scan_check['safe']:raise RuntimeError(scan_check['reason'])
        return {'map':evidence,'scan':scan_check,'braking_extension_m':braking}

    def monitor_short_motion(self):
        ctx=getattr(self,'short_motion_active',None)
        if not ctx or self.canceling or self.action_result is None or self.action_result.done():return
        now=time.monotonic();p=self.recovery_feedback();a=ctx['odom_start']
        dx,dy=p[0]-a[0],p[1]-a[1];c,s=math.cos(a[2]),math.sin(a[2])
        forward=dx*c+dy*s;travel=ctx['direction']*forward
        if (abs(-dx*s+dy*c)>POLICY['lateral_tolerance_m']
                or abs(angle(p[2],a[2]))>POLICY['heading_tolerance_rad']
                or travel < -POLICY['endpoint_tolerance_m']
                or travel>ctx['distance']+POLICY['endpoint_tolerance_m']):
            raise RuntimeError('short_motion_route_deviation')
        if self.distance-ctx['distance_start']>ctx['distance']+2*POLICY['endpoint_tolerance_m']:
            raise RuntimeError('short_motion_distance_limit')
        if now-ctx['last_progress_at']>POLICY['no_progress_wall_s']:
            raise RuntimeError('short_motion_no_progress')
        if travel>ctx['progress']+.005:
            ctx.update(progress=travel,last_progress_at=now)
        if now-ctx['last_check_at']>=.1:
            if self.ensure_epoch()!=ctx['epoch']:raise RuntimeError('map_epoch_changed')
            # Recheck against current map and scans; guard independently checks every velocity.
            self.short_clearance(ctx['direction']*max(.001,ctx['distance']-travel))
            ctx['last_check_at']=now

    def short_action(self, signed_distance, epoch, audit):
        from rclpy.action import ActionClient
        from nav2_msgs.action import BackUp, DriveOnHeading
        self.health();self.require_recovery_contract()
        if self.handle is not None or not self.stop_confirmed():raise RuntimeError('stop_unconfirmed')
        if self.ensure_epoch()!=epoch:raise RuntimeError('map_epoch_changed')
        reverse=signed_distance<0;action=BackUp if reverse else DriveOnHeading
        client=ActionClient(self.node,action,'/backup' if reverse else '/drive_on_heading')
        try:
            if not client.wait_for_server(timeout_sec=3):raise RuntimeError('short_motion_action_unavailable')
            goal=action.Goal();goal.target.x=float(signed_distance);goal.speed=POLICY['speed_m_s']
            # Older action schemas do not expose a bypass flag; default behavior
            # still checks collisions. Never set/implement a bypass on any version.
            if hasattr(goal,'disable_collision_checks'):goal.disable_collision_checks=False
            allowance=abs(signed_distance)/POLICY['speed_m_s']+5.
            goal.time_allowance.sec=math.ceil(allowance)
            from robot_skills import mission
            self.store.update(self.identifier,distance_odom_m=self.distance)
            m=self.store.meta('mission:'+self.task['mission_id'])
            budget=mission.action_budget(m,self.store.tasks(),self.node.get_clock().now().nanoseconds*1e-9,
                                        allowance,abs(signed_distance)+POLICY['endpoint_tolerance_m'])
            if not budget['fits']:raise RuntimeError('short_motion_budget_exhausted')
            if m['deadline_monotonic_s']-time.monotonic()<POLICY['action_wall_timeout_s']:
                raise RuntimeError('short_motion_wall_budget_exhausted')
            audit.update(requested_distance_m=signed_distance,speed_m_s=goal.speed,budget=budget,
                         collision_checks=True,clearance=self.short_clearance(signed_distance))
            odom=self.recovery_feedback();now=time.monotonic()
            self.short_motion_active={'odom_start':odom,'direction':-1 if reverse else 1,
                'distance':abs(signed_distance),'distance_start':self.distance,'epoch':epoch,
                'last_check_at':now,'last_progress_at':now,'progress':0.}
            self.store.update(self.identifier,phase='recovering' if reverse else 'probing',short_motion=audit)
            self.health()
            outcome=self.execute(client,goal,POLICY['action_wall_timeout_s'],'short_motion')
            # Execute has observed terminal result and stopped; late-goal cleanup
            # must only track an actually unresolved send, not an old completed goal.
            self.pending_goal=None
            end=self.recovery_feedback()
            delta=(end[0]-odom[0])*math.cos(odom[2])+(end[1]-odom[1])*math.sin(odom[2])
            lateral=-(end[0]-odom[0])*math.sin(odom[2])+(end[1]-odom[1])*math.cos(odom[2])
            if outcome['status']==4 and (abs(delta-signed_distance)>POLICY['endpoint_tolerance_m']
                    or abs(lateral)>POLICY['lateral_tolerance_m']
                    or abs(angle(end[2],odom[2]))>POLICY['heading_tolerance_rad']):
                outcome={'status':6,'error_msg':'short_motion_endpoint_not_reached'}
            audit.update(outcome=outcome,actual_signed_distance_m=delta,stopped=True)
            return outcome
        finally:
            self.short_motion_active=None
            # Retain the action client until Executor.close: an unresolved goal
            # must remain cancelable by outer cleanup after a late acceptance.
            self.short_clients.append(client)

    def retreat(self, result, source_id, trace, stationary_bridge=None):
        epoch=self.ensure_epoch();attempt=reserve(self.store,self.task,self.pose(),epoch,source_id)
        audit={'attempt':attempt['attempt'],'source_task_id':source_id}
        result.setdefault('recoveries',[]).append(audit)
        start_distance=self.distance;start=time.monotonic()
        sim_start=self.node.get_clock().now().nanoseconds*1e-9
        try:
            self.health()
            distance=straight_retreat(trace,self.recovery_feedback(),self.node.get_clock().now().nanoseconds*1e-9,stationary_bridge)
            update_attempt(self.store,self.task['mission_id'],attempt['attempt'],status='checking',distance_m=distance)
            outcome=self.short_action(-distance,epoch,audit)
            audit['status']='succeeded' if outcome['status']==4 else 'failed'
            if outcome['status']==4:
                self.observe(.5)
                audit['new_map_version']=self.latest['map_version']
                audit['next_action']='replan_or_change_approach; no_automatic_forward_retry'
            return outcome
        except Exception as error:
            audit.update(status='failed' if self.handle is not None else 'denied',reason=str(error))
            raise
        finally:
            audit.update(distance_odom_m=self.distance-start_distance,elapsed_wall_s=time.monotonic()-start,
                         elapsed_sim_s=self.node.get_clock().now().nanoseconds*1e-9-sim_start)
            update_attempt(self.store,self.task['mission_id'],attempt['attempt'],**audit)

    def automatic_retreat(self, result):
        """One reverse after a confirmed failed action; no forward/reverse retry loop."""
        if (not self.two_stage or not self.dispatched or not eligible(result.get('reason'))
                or result.get('recoveries') or self.task['kind']=='recover_short_reverse'):
            return
        self.canceling=True
        try:
            # A pending acceptance cannot be treated as a terminal action.
            if self.handle is None and self.pending_goal is not None:
                handle=self.wait(self.pending_goal,3)
                if handle.accepted:
                    self.handle,self.action_result=handle,handle.get_result_async()
            if not self.cancel_active():raise RuntimeError('stop_unconfirmed')
            self.pending_goal=None
        finally:
            self.canceling=False
        self.health()
        self.retreat(result,self.identifier,list(self.odom_trace))
        # Query a fresh route to the original target as feedback, without moving
        # forward again. The persistent task decides whether to change approach.
        candidate=self.active_candidate
        if candidate and result['recoveries'][-1].get('status')=='succeeded':
            plan,outcome=self.plan(candidate)
            result['recovery_replan']={'planning':outcome,'route_available':plan is not None}

    def perform_short_motion(self, result):
        payload=self.task['payload'];epoch=self.ensure_epoch()
        if epoch!=payload['map_epoch']:raise RuntimeError('map_epoch_changed')
        result['map_epoch']=epoch
        inv=self.store.meta('investigation:'+payload['investigation_id'],{})
        m=self.store.meta('mission:'+self.task['mission_id'],{})
        if (inv.get('status')!='active' or inv.get('map_epoch')!=epoch
                or m.get('active_investigation')!=payload['investigation_id'] or m.get('phase')!='ai_investigation'):
            raise RuntimeError('active_investigation_required')
        if self.task['kind']=='recover_short_reverse':
            source=self.store.get(payload['source_task_id'])
            why=source_reason(source,self.store.tasks(),self.task['mission_id'],epoch,payload['investigation_id'])
            if why:raise RuntimeError(why)
            trace=source['result'].get('odom_trace',[])
            current=list(self.odom_trace)
            if not trace or not current:raise RuntimeError('recovery_trace_unavailable')
            if trace[-1]['frame']!=current[0]['frame']:raise RuntimeError('recovery_trace_frame_changed')
            bridge=(trace[-1]['sim_s'],current[0]['sim_s'])
            # ready() confirmed stopped; source stopped is required by admission.
            # straight_retreat verifies both boundary poses coincide, preserving
            # original timestamps and rejecting a moving/unobserved connector.
            outcome=self.retreat(result,source['task_id'],trace+current,bridge)
        else:
            why=probe_reason(self.store.tasks(),self.task['mission_id'],self.pose(),epoch)
            if why:raise RuntimeError(why)
            denied=[];distance=None
            for step in POLICY['probe_steps_m']:
                try:
                    self.health();self.short_clearance(step);distance=step;break
                except RuntimeError as error:
                    if str(error) not in ('body_sweep_nonfree','body_outside_known_map','scan_body_sweep'):raise
                    denied.append({'distance_m':step,'reason':str(error)})
            result['probe']={'shorter_step_checks':denied}
            if distance is None:raise RuntimeError(denied[-1]['reason'])
            p=self.pose()
            candidate={'x':p[0]+distance*math.cos(p[2]),'y':p[1]+distance*math.sin(p[2]),
                       'yaw':p[2],'region_epoch':epoch}
            result['candidate']=candidate
            self.store.update(self.identifier,candidate=candidate)
            outcome=self.short_action(distance,epoch,result['probe'])
        result.update(status='succeeded' if outcome['status']==4 else 'aborted',
                      reason='short_motion_completed' if outcome['status']==4 else outcome.get('error_msg') or 'short_motion_failed')
