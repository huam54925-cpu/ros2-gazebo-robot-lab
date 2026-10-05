"""ROS executor mixin for finite map-checked observation choices."""
import math
import time
import uuid
import numpy as np
from nav2_msgs.action import Spin
from explore import yaw, rclpy
from safety_contract import BASE_FRAME, SCAN_FRAME, STOP_RADIUS_M
from observation import (navigation_footprint, rotation_path, check_rotation, Visibility,
                         estimated_time, map_change, navigation_prediction)
from path_safety import angle_delta
from robot_skills.observation_rules import exclusion


class ObservationMixin:
    def grid_snapshot(self):
        m=self.latest['map']; o=m.info.origin
        return {'data':self.latest['data'].copy(),'resolution':m.info.resolution,
                'origin':(o.position.x,o.position.y,yaw(o.orientation)), 'frame':m.header.frame_id,
                'stamp_sim_s':m.header.stamp.sec+m.header.stamp.nanosec*1e-9}

    def sensor_offset(self):
        try: t=self.buffer.lookup_transform(BASE_FRAME,SCAN_FRAME,rclpy.time.Time())
        except Exception as error: raise RuntimeError('lidar_transform_unavailable') from error
        return (t.transform.translation.x,t.transform.translation.y)

    def rotation_check(self,pose,angle):
        grid=self.grid_snapshot(); footprint,padding=navigation_footprint()
        return check_rotation(grid['data'],grid['resolution'],grid['origin'],self.latest['map_version'],
                              pose,angle,self.sensor_offset(),footprint,padding)

    def observation_catalog(self):
        self.ready(); start=self.pose(); grid=self.grid_snapshot(); offset=self.sensor_offset()
        gain_version=self.latest['map_version']
        visibility=Visibility(grid['data'],grid['resolution'],grid['origin'],offset,self.latest.get('scan_range_max_m'))
        tasks=self.store.tasks(); recent=[t['result']['after']['pose'] for t in reversed(tasks)
                 if t.get('result') and t['result'].get('after')][-8:]
        options=[]; rejected=[]; batch=uuid.uuid4().hex[:12]; deadline=time.monotonic()+45
        def add(candidate,poses,length,clearance):
            from regions import region_id
            epoch=self.store.meta('regional_epoch',{})
            if epoch.get('geometry')==[grid['frame'],grid['resolution'],grid['origin'][2]]:
                candidate.update(region_epoch=epoch['id'],region_id=region_id([candidate['x'],candidate['y']]),
                                 target_region_id=region_id([candidate['x'],candidate['y']]))
            sampled=[*poses[::max(1,len(poses)//12)],poses[-1]]
            gain=visibility.novel(sampled,start,recent)
            duration=estimated_time(length,poses)
            candidate.update(start_pose=start,map_version=clearance['map_version'],gain_map_version=gain_version,safe=True,
                novel_unknown_area_proxy_m2=gain,gain_model='sensor_offset_occlusion_aware_rays_v2',
                unknown_analysis=visibility.analyze(poses[-1],start),
                gain_confidence='optimistic_proxy',estimated_sim_seconds=duration,
                estimated_wall_seconds=None,planned_length_m=length,clearance=clearance,score=gain/duration,
                option_id='OBS_'+batch+'_'+str(len(options)+1))
            why=exclusion(candidate,tasks)
            if why or gain<.25:
                rejected.append({'type':candidate['type'],'reason':why or 'insufficient_novelty_proxy'}); return
            options.append(candidate)
        for angle in (math.pi/4,-math.pi/4,math.pi/2,-math.pi/2):
            self.health()
            check=self.rotation_check(start,angle)
            if not check['safe']:
                rejected.append({'type':'rotate','angle_rad':angle,'reason':check['reason']}); continue
            poses=rotation_path(start,angle,.2) # Separate coarse information samples, never safety samples.
            add({'type':'rotate','signed_angle_rad':angle,'x':start[0],'y':start[1],'yaw':start[2]+angle},poses,0.,check)
        model=self.model(); candidates=[]
        for distance in (.75,1.25):
            for bearing in np.linspace(-math.pi,math.pi,8,endpoint=False):
                x,y=start[0]+distance*math.cos(bearing),start[1]+distance*math.sin(bearing)
                if not model.is_safe(x,y,bearing): continue
                candidate={'type':'move_observe','x':x,'y':y,'yaw':float(bearing),'start_pose':start}
                if exclusion(candidate,tasks): continue
                gain=visibility.novel([start,[x,y,bearing]],start,recent)
                candidates.append((gain,candidate))
        for _,candidate in sorted(candidates,key=lambda p:p[0],reverse=True)[:4]:
            if time.monotonic()>deadline: break
            self.health(); plan,outcome=self.plan(candidate)
            if plan is None or plan['length']>2:
                rejected.append({'type':'move_observe','reason':'no_short_safe_path'}); continue
            poses=navigation_prediction(plan['path'],start,[candidate['x'],candidate['y'],candidate['yaw']])
            add(candidate,poses,plan['length'],plan['clearance'])
        self.health()
        result={'catalog_id':batch,'created_unix_s':time.time(),'expires_unix_s':time.time()+120,
                'map_version':self.latest['map_version'],'start_pose':start,'options':options,'rejected':rejected,
                'status':'ready' if options else 'no_safe_observation_options',
                'requires_execution_revalidation':True,'sensor_fresh':self.freshness()}
        self.store.set_meta('observation_catalog',result); return result

    def prepare_observation(self,candidate):
        catalog=self.store.meta('observation_catalog',{})
        if time.time()>catalog.get('expires_unix_s',0): raise RuntimeError('observation_catalog_expired')
        if not any(c['option_id']==candidate['option_id'] for c in catalog.get('options',[])):
            raise RuntimeError('observation_no_longer_exists')
        if exclusion(candidate,[t for t in self.store.tasks() if t['task_id']!=self.identifier]):
            raise RuntimeError('observation_repeat_blocked')
        if math.dist(self.pose()[:2],candidate['start_pose'][:2])>.15 or abs(angle_delta(self.pose()[2],candidate['start_pose'][2]))>.15:
            raise RuntimeError('observation_start_pose_changed')

    def perform_rotation(self,candidate,result):
        if not self.spin_client.wait_for_server(timeout_sec=5): raise RuntimeError('spin_unavailable')
        angle=candidate['signed_angle_rad']
        if abs(angle) not in (math.pi/4,math.pi/2): raise RuntimeError('invalid_local_rotation_option')
        for _ in range(3):
            self.health(); version=self.latest['map_version']; start=self.pose()
            check=self.rotation_check(start,angle); result['clearance']=check
            if not check['safe']: raise RuntimeError(check['reason'])
            self.health()
            if version==self.latest['map_version']: break
        else: raise RuntimeError('map_unstable_during_validation')
        if self.latest['nearest']<STOP_RADIUS_M: raise RuntimeError('guard_blocked_no_safe_motion')
        goal=Spin.Goal(); goal.target_yaw=float(angle); goal.disable_collision_checks=False
        goal.time_allowance.sec=math.ceil(abs(angle)/.15+10)
        self.store.update(self.identifier,phase='observing')
        self.dispatched=True; self.pending_goal=self.spin_client.send_goal_async(goal)
        self.handle=self.wait(self.pending_goal,10)
        if not self.handle.accepted:
            self.handle=None; raise RuntimeError('spin_rejected')
        self.action_result=self.handle.get_result_async()
        previous=start[2]; traveled=0.; checked_at=0.; checked_version=None; deadline=time.monotonic()+110
        while not self.action_result.done():
            self.spin_once(); self.health(); pose=self.pose(); now=time.monotonic()
            delta=angle_delta(previous,pose[2]); previous=pose[2]
            if abs(delta)>.35 or math.dist(start[:2],pose[:2])>.15: raise RuntimeError('rotation_pose_discontinuity')
            traveled+=delta
            if self.latest['nearest']<STOP_RADIUS_M: raise RuntimeError('guard_stop')
            remaining=angle-traveled
            if abs(traveled)>abs(angle)+.15: raise RuntimeError('rotation_overshoot')
            if now>deadline: raise RuntimeError('observation_timeout')
            if now-checked_at>.5 or checked_version!=self.latest['map_version']:
                check=self.rotation_check(pose,remaining)
                if not check['safe']: raise RuntimeError('rotation_path_invalidated:'+check['reason'])
                checked_at=now; checked_version=self.latest['map_version']
        outcome=self.action_result.result(); self.handle=None; self.action_result=None
        result.update(navigation_result={'status':outcome.status,'error_code':outcome.result.error_code,
                                         'error_msg':outcome.result.error_msg},
                      status='succeeded' if outcome.status==4 else 'aborted',
                      reason='observation_rotation_completed' if outcome.status==4 else 'spin_aborted',
                      signed_angle_traveled_rad=traveled)

    def observation_feedback(self):
        at_stop=self.grid_snapshot(); self.save_map('gain_stopped')
        sim_start=self.node.get_clock().now().nanoseconds*1e-9; wall=time.monotonic(); fresh=True
        while time.monotonic()-wall<20 and self.node.get_clock().now().nanoseconds*1e-9-sim_start<4:
            self.spin_once()
            try: self.health()
            except RuntimeError: fresh=False; break
            if self.store.meta('stop_latched',False): break
            if self.task.get('mission_id'):
                m=self.store.meta('mission:'+self.task['mission_id'])
                if m['status']!='running' or time.monotonic()>m['deadline_monotonic_s']: break
        after=self.grid_snapshot(); self.save_map('gain_settled')
        metrics=map_change(self.metric_before,after,allow_fractional=True)
        metrics.update(at_stop=map_change(self.metric_before,at_stop,allow_fractional=True),settling=map_change(at_stop,after,allow_fractional=True),
                       feedback_wait_wall_s=time.monotonic()-wall,sensor_feedback_fresh=fresh,
                       map_message_advanced=after['stamp_sim_s']>self.metric_before['stamp_sim_s'])
        metrics['usable_for_trend']=bool(fresh and metrics['map_message_advanced'] and
            metrics.get('registration')=='integer_grid' and metrics.get('observed_lost_known_area_m2',float('inf'))<.05)
        metrics['causal_action_gain_valid']=False
        metrics['trend_scope']='registered_map_window_only_not_causal_action_gain'
        metrics['before_map_stamp_sim_s']=self.metric_before['stamp_sim_s']
        metrics['after_map_stamp_sim_s']=after['stamp_sim_s']
        metrics['before_geometry']={k:v for k,v in self.metric_before.items() if k!='data'}
        metrics['after_geometry']={k:v for k,v in after.items() if k!='data'}
        return metrics
