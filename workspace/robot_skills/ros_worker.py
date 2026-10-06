"""ROS-side deterministic executor. Never imports the OpenAI client or reads keys."""
import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time
from types import SimpleNamespace as NS
import uuid

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(WORKSPACE), str(WORKSPACE / 'robot_agent'),
               str(WORKSPACE / 'navigation_base/exploration'), str(WORKSPACE / 'navigation_base')]
from robot_skills.store import Store, DIRECTORY
from motion_worker import Trial
from explore import Explorer, rclpy, GetState, NavigateToPose, ManageLifecycleNodes
from aggressive import utility
from rclpy.signals import SignalHandlerOptions
from action_msgs.srv import CancelGoal
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


from robot_skills.observation_worker import ObservationMixin


from robot_skills.investigation_worker import InvestigationMixin
from robot_skills.investigation import QUERY_KINDS, NAVIGATION_KINDS, failure_code


class Executor(InvestigationMixin, ObservationMixin, Trial):
    def odom_cb(self,msg):
        super().odom_cb(msg)
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        previous=getattr(self,'timed_odom',None)
        navigating=getattr(getattr(self,'phase_timer',None),'phase',None)=='navigation_including_turns'
        if navigating and previous and previous[3] and 0<stamp-previous[0]<=.5:
            dt=stamp-previous[0];v,w=previous[1:3]
            key='translation_including_curves' if abs(v)>=.02 else 'turn_in_place' if abs(w)>=.02 else 'stationary'
            self.motion_seconds[key]=self.motion_seconds.get(key,0.)+dt
            if abs(v)>=.02 and abs(w)>=.02:
                self.motion_seconds['turn_while_translating']=self.motion_seconds.get('turn_while_translating',0.)+dt
        self.timed_odom=(stamp,msg.twist.twist.linear.x,msg.twist.twist.angular.z,navigating)

    def scan_cb(self,msg):
        Explorer.scan_cb(self,msg)
        from robot_skills.scan_diagnostics import summarize_scan
        self.latest['scan_statistics']=summarize_scan(msg.ranges,msg.range_min,msg.range_max)

    def __init__(self, store, task=None):
        self.store, self.task = store, task
        self.identifier = task['task_id'] if task else str(uuid.uuid4())
        self.cancel_path = DIRECTORY / (self.identifier + '.unused')
        self.dispatched, self.pending_goal, self.start_pose = False, None, None
        self.canceling = False
        self.last_status_at = 0
        self.fixed = bool(task and task['kind'] == 'fixed_step')
        self.mission_state=store.meta('mission:'+str((task or {}).get('mission_id')), {})
        self.two_stage=self.mission_state.get('two_stage',False)
        Explorer.__init__(self, NS(output=str(DIRECTORY / (self.identifier + '.detail.json')),
                                  policy='aggressive', wall_budget=240, max_goals=1,
                                  goal_timeout=45 if self.fixed else 180))
        from robot_skills.exploration_upgrade import search_for,short_start_for,path_clearance_for
        self.candidate_search=(task.get('candidate_search','baseline') if task else search_for(store))
        self.required_path_clearance_m=path_clearance_for(store,(task or {}).get('mission_id'))
        self.short_start=short_start_for(store,task)
        if self.candidate_search=='wide_4_5':
            from dataclasses import replace,asdict
            self.config=replace(self.config,maximum_frontier_distance=4.5,
                                minimum_goal_distance=.4 if self.short_start else 1.2)
            self.report['configuration']=asdict(self.config)
        if self.two_stage:
            from dataclasses import replace,asdict
            self.config=replace(self.config,maximum_frontier_distance=4.5,maximum_candidates=24)
            self.report['configuration']=asdict(self.config)
            self.args.wall_budget=max(1,self.mission_state['deadline_monotonic_s']-time.monotonic())
        self.report['candidate_search']=self.candidate_search
        self.report['short_start']=self.short_start
        self.report['path_clearance_m']=self.required_path_clearance_m
        from safety_profile import FOOTPRINT_MODE
        if FOOTPRINT_MODE and not self.fixed:
            self.args.goal_timeout=600
            self.report['limits']['goal_wall_timeout']=600
        from robot_skills.provenance import source_manifest
        self.report['source_sha256'] = source_manifest()

    def freshness(self):
        now = time.monotonic()
        result = {name: now-self.latest.get(key, -math.inf) < limit for name, key, limit in
                  [('odometry', 'odom_at', 2), ('scan', 'scan_at', 1.5), ('map', 'map_at', 20)]}
        result['scan'] &= self.latest.get('scan_valid', False)
        try:
            self.pose(); result['localization'] = True
        except Exception:
            result['localization'] = False
        return result

    def health(self):
        try:
            Explorer.health(self)
        except RuntimeError as error:
            if str(error) not in ('localization_stale', 'localization_unavailable'):
                raise
            # Geometry work can queue TF behind fresh sensor callbacks. Drain
            # already delivered callbacks once; never increase TF age limits.
            for _ in range(128):
                rclpy.spin_once(self.node, timeout_sec=0.)
            Explorer.health(self)
        if self.task and not self.canceling:
            if self.store.should_cancel(self.identifier):
                raise RuntimeError('stop_requested')
            if self.distance > (.65 if self.fixed else self.mission_state['limits']['max_distance'] if self.two_stage else 10):
                raise RuntimeError('distance_budget_exhausted')
            if time.monotonic()-self.last_status_at >= .5:
                if self.task.get('mission_id'):
                    from robot_skills import mission
                    m = self.store.meta('mission:'+self.task['mission_id'])
                    tasks = self.store.tasks()
                    for task in tasks:
                        if task['task_id'] == self.identifier: task['distance_odom_m'] = self.distance
                    why = mission.reason(m, tasks, self.node.get_clock().now().nanoseconds*1e-9, check_steps=False)
                    if why: raise RuntimeError(why)
                self.store.update(self.identifier, sensor_fresh=self.freshness(),
                                  map_version=self.latest.get('map_version'), distance_odom_m=self.distance)
                self.last_status_at = time.monotonic()

    def ready(self):
        from safety_profile import FOOTPRINT_MODE
        if FOOTPRINT_MODE:
            guard_path=WORKSPACE/'log'/'footprint-guard-status.json'
            guard=json.loads(guard_path.read_text())
            from navigation_base.robot_contract import CONTRACT_HASH
            if (guard.get('profile')!='footprint_075' or time.monotonic()-guard.get('generated_monotonic_s',0)>2
                    or (self.two_stage and guard.get('contract_hash')!=CONTRACT_HASH)):
                raise RuntimeError('footprint_guard_profile_not_confirmed')
        for client in (self.planner, self.nav):
            if not client.wait_for_server(timeout_sec=10):
                raise RuntimeError('nav2_unavailable')
        for name in ('planner_server', 'controller_server', 'behavior_server', 'bt_navigator'):
            client = self.node.create_client(GetState, '/' + name + '/get_state')
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError(name + '_unavailable')
            if self.wait(client.call_async(GetState.Request()), 5).current_state.id != 3:
                raise RuntimeError(name + '_not_active')
            self.node.destroy_client(client)
        deadline = time.monotonic()+20
        while not all(k in self.latest for k in ('map', 'scan_at', 'odom_at')) or not self.buffer.can_transform('map', 'vehicle/base_link', rclpy.time.Time()):
            if time.monotonic() > deadline:
                raise RuntimeError('missing_initial_feedback')
            self.spin_once()
        self.observe(1)
        if any(s.status in (1, 2, 3) for s in self.latest.get('status', NS(status_list=[])).status_list):
            raise RuntimeError('another_navigation_is_active')
        if not self.stop_confirmed():
            raise RuntimeError('robot_not_stationary')
        self.health()

    def cancel_active(self):
        if self.handle is None:
            return self.stop_confirmed()
        try:
            response = self.wait(self.handle.cancel_goal_async(), 8)
            outcome = self.wait(self.action_result, 10)
            self.event('canceled_action', accepted=bool(response.goals_canceling), status=outcome.status)
            if outcome.status not in (4, 5, 6):
                raise RuntimeError('action_not_terminal')
            self.handle = self.action_result = None
            if self.stop_confirmed():
                return True
        except Exception as error:
            self.event('cancel_error', message=str(error))
        # Zero odometry alone cannot prove an outstanding goal is disabled.
        paused = self.pause_navigation()
        self.handle = self.action_result = None
        return bool(paused and self.stop_confirmed())

    def close(self):
        self.event_file.close()
        self.node.destroy_node()
        self.lock.close()
        self.motion_lock.close()

    def upgrade_bridge(self,snapshot,pose,epoch,mid,mode):
        from robot_skills.upgrade_geometry import UpgradeGeometry
        from safety_contract import BASE_FRAME,SCAN_FRAME
        from explore import yaw
        t=self.buffer.lookup_transform(BASE_FRAME,SCAN_FRAME,rclpy.time.Time())
        bridge=UpgradeGeometry(self.store,mid,epoch,snapshot,pose,
                               (t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)),
                               self.latest.get('scan_range_max_m'),mode)
        bridge.scan_statistics=self.latest.get('scan_statistics',{})
        return bridge

    def evaluate_path(self,path,candidate):
        result=Explorer.evaluate_path(self,path,candidate)
        if getattr(self,'candidate_search','baseline')=='wide_4_5' and result['safe'] and 'body_sweep' not in result:
            from observation import body_sweep
            grid=self.grid_snapshot()
            body=body_sweep(grid['data'],grid['resolution'],grid['origin'],path,self.pose(),
                            [candidate['x'],candidate['y'],candidate['yaw']])
            result={**result,'body_sweep':body,'safe':body['safe'],
                    'reason':'clear' if body['safe'] else 'body_sweep_nonfree'}
        self.last_clearance=result
        return result

    def catalog(self):
        self.ready()
        from robot_skills.exploration_upgrade import mode_for,search_for
        mode=mode_for(self.store);mid=self.store.meta('active_mission')
        search=search_for(self.store)
        excluded = [(t['candidate']['x'], t['candidate']['y']) for t in self.store.tasks()
                    if t.get('candidate') and t['accepted']]
        if mode=='memory' or self.two_stage:excluded=[] # Policy memory replaces permanent visited circles, never safety geometry.
        previous=self.store.meta('catalog',{})
        offset=(previous.get('next_candidate_offset',0) if not previous.get('candidates') and
                previous.get('map_version')==self.latest['map_version'] and previous.get('more_candidates') and
                previous.get('candidate_search','baseline')==search and
                previous.get('path_clearance_m',2.15)==self.required_path_clearance_m and
                previous.get('exploration_upgrade',{}).get('mode','baseline')==mode and
                (mode=='baseline' or previous.get('exploration_upgrade',{}).get('mission_id')==mid) else 0)
        safe, rejected = [], []
        from observation import Visibility, estimated_time, action_time_estimate, navigation_prediction, map_semantics
        from regions import region_id, tier, transit_candidates, transit_exclusions
        grid = self.grid_snapshot()
        signature = [grid['frame'], grid['resolution'], grid['origin'][2]]
        memory = self.store.meta('regional_epoch', {})
        if memory.get('geometry') != signature or grid['stamp_sim_s'] < memory.get('last_sim_s', 0):
            memory = {'id': uuid.uuid4().hex[:12], 'geometry': signature}
        memory['last_sim_s'] = grid['stamp_sim_s']
        self.store.set_meta('regional_epoch', memory)
        visibility = Visibility(grid['data'], grid['resolution'], grid['origin'], self.sensor_offset(),
                                self.latest.get('scan_range_max_m'))
        start = self.pose()
        gain_version = self.latest['map_version']
        bridge=self.upgrade_bridge(grid,start,memory['id'],mid,mode) if mode!='baseline' else None
        generation_audit=[]
        model=self.model()
        from safety_profile import FOOTPRINT_MODE
        def drain_feedback():
            for _ in range(64):rclpy.spin_once(self.node,timeout_sec=0.)
        candidates, info = model.candidates(start, excluded, regional=True,candidate_offset=offset,
                                             **({'progress':drain_feedback} if FOOTPRINT_MODE else {}),
                                             **({'audit':generation_audit} if bridge else {}))
        if bridge and mode=='memory':
            from frontier import components
            from exploration_support.frontier_identity import associate, candidate_identity
            key='frontier_identity:'+mid+':'+memory['id']
            groups=[model.world(g) for g in components(model.frontier)
                    if len(g)*model.resolution>=model.settings.minimum_frontier_length]
            identities=associate(groups,self.store.meta(key,{}))
            self.store.set_meta(key,identities)
            for candidate in candidates: candidate.update(candidate_identity(candidate,identities))
            info['frontier_identity']={**identities,'clusters':[
                {k:v for k,v in c.items() if k!='points'} for c in identities['clusters']]}
        raw_candidates=json.loads(json.dumps(candidates)) if bridge else None
        checked_paths={};plan_rejections=[]
        recent = [t['result']['after']['pose'] for t in reversed(self.store.tasks())
                  if t.get('result') and t['result'].get('after')
                  and (t.get('candidate') or {}).get('region_epoch')==memory['id']][-8:]
        batch = uuid.uuid4().hex[:12]
        deadline = time.monotonic()+45
        attempted = 0
        for candidate in candidates:
            if len(safe) >= 6 or time.monotonic() > deadline:
                break
            self.health()
            attempted += 1
            if self.task and self.task['kind']=='search_frontiers':
                self.store.update(self.identifier,phase='searching',search_progress={
                    'checked':offset+attempted,'shortlist_size':len(candidates),
                    'total_proposals':info.get('oriented_candidates_before_cap'),
                    'safe_found':len(safe),'search_complete':False})
            candidate.update(region_epoch=memory['id'], target_region_id=candidate['region_id'], transit=False)
            if self.two_stage:
                from robot_skills.investigation import repetition_reason
                blocked=repetition_reason(self.store.meta('investigation_memory:'+mid,[]),[candidate],memory['id'],start)
                if blocked:
                    rejected.append({'candidate_id':candidate['id'],'reason':blocked});continue
            plan, outcome = self.plan(candidate)
            if plan is None:
                if bridge:plan_rejections.append((dict(candidate),outcome))
                rejected.append({'candidate_id': candidate['id'], 'reason': outcome,
                                 'target_region_id':candidate['region_id'],
                                 'observation_pose':[candidate['x'],candidate['y'],candidate['yaw']],
                                 'unknown_analysis':candidate.get('unknown_analysis'),
                                 'rejection_scope':'this_sampled_pose_and_planned_path',
                                 'recheck_on_map_change':True,'region_proven_unreachable':False})
                continue
            regional_length = plan['length']
            goal = [candidate['x'],candidate['y'],candidate['yaw']]
            if mode=='memory':
                advice=bridge.advise(candidate,navigation_prediction(plan['path'],start,goal))
                if not advice['retain_for_safety_recheck']:
                    bridge.audit.append({'candidate_id':candidate['id'],'stage':'regional_target_policy',
                                         'mode':mode,'policy':advice,'purpose':'observe_goal'})
                    bridge.plans.append({'candidate':dict(candidate),'checked_path':plan['path'],'policy':advice})
                    rejected.append({'candidate_id':candidate['id'],'stage':'exploration_policy','reason':advice['reason']})
                    continue
            potential = visibility.novel([goal], start, recent)
            target_unknown = visibility.analyze(goal,start)
            if plan['length'] > 8:
                blocked = [] if mode=='memory' else transit_exclusions(self.store.tasks(), candidate)
                relays = transit_candidates(candidate, plan['path'], start, blocked)
                attempts = []
                selected = None
                for relay in relays:
                    if time.monotonic() > deadline:
                        break
                    self.health()
                    if not self.model().is_safe(relay['x'], relay['y'], relay['yaw']):
                        attempts.append({'pose':[relay['x'],relay['y'],relay['yaw']], 'reason':'unsafe_pose'})
                        continue
                    relay_plan, outcome = self.plan(relay)
                    if relay_plan is None or relay_plan['length'] > 8:
                        attempts.append({'pose':[relay['x'],relay['y'],relay['yaw']],
                                         'reason':'transit_path_rejected', 'planning':outcome})
                        continue
                    selected = (relay, relay_plan)
                    break
                if selected is None:
                    if len(attempts) < len(relays):
                        # Retry this candidate in the next read-only batch;
                        # unfinished validation is not an unreachable result.
                        attempted -= 1
                        break
                    rejected.append({'candidate_id':candidate['id'], 'reason':'no_safe_transit_pose',
                                     'transit_attempts':attempts, 'transit_options':len(relays)})
                    continue
                candidate, plan = selected
            if any(math.dist([candidate['x'],candidate['y']], [c['x'],c['y']]) < .8 for c in safe):
                rejected.append({'candidate_id':candidate['id'], 'reason':'duplicate_safe_endpoint'})
                continue
            poses = navigation_prediction(plan['path'], start, [candidate['x'],candidate['y'],candidate['yaw']])
            sampled = [*poses[::max(1,len(poses)//12)], poses[-1]]
            novel = visibility.novel(sampled, start, recent)
            estimate = action_time_estimate(plan['length'], poses, grid['resolution']) if mode=='memory' else None
            duration = estimate['budget_sim_s'] if estimate else estimated_time(plan['length'], poses)
            item={**candidate, 'gain_map_version': gain_version, 'novel_unknown_area_proxy_m2': novel,
                         'unknown_analysis':visibility.analyze([candidate['x'],candidate['y'],candidate['yaw']],start),
                         'regional_target_unknown_analysis':target_unknown,
                         'gain_model':'sensor_offset_occlusion_aware_rays_v2',
                         'distance_tier':tier(plan['length']), 'regional_route_length_m':regional_length,
                         'target_distance_tier':tier(regional_length),
                         'regional_potential_proxy_m2':potential,
                         'regional_score':max(novel,potential)/(max(duration,regional_length/.15)+20),
                         'estimated_sim_seconds': duration, 'action_time_estimate':estimate, 'joint_score': novel/duration,
                         'frontier_id': 'F_'+batch+'_'+str(attempted if mode=='memory' else len(safe)+1),
                         'map_version': plan['clearance']['map_version'], 'planned_length_m': plan['length'],
                         'clearance': plan['clearance'],
                         'classical_score': utility(candidate, plan['length'], self.pose()[2])}
            if mode=='memory':
                item,retain=bridge.enrich(item,poses)
                if not retain:
                    rejected.append({'candidate_id':candidate['id'],'stage':'exploration_policy',
                                     'reason':item['exploration_upgrade']['policy']['reason'],
                                     'exploration_upgrade':item['exploration_upgrade']})
                    continue
            safe.append(item);checked_paths[item['frontier_id']]=poses
        self.health()
        more=offset+attempted < info.get('oriented_candidates_before_cap',len(candidates))
        complete = (attempted == len(candidates) and not more and gain_version == self.latest['map_version']
                    if self.two_stage else offset==0 and attempted == len(candidates)
                    and not info.get('shortlist_truncated',False) and gain_version == self.latest['map_version'])
        result = {'catalog_id': batch, 'catalog_schema':3, 'region_epoch':memory['id'],
                  'candidate_search':search,'short_start':getattr(self,'short_start',False),
                  'path_clearance_m':getattr(self,'required_path_clearance_m',2.15),
                  'minimum_goal_distance_m':self.config.minimum_goal_distance,
                  'unknown_space':{**map_semantics(grid['data'],grid['resolution']),
                      'analysis_map_version':gain_version,
                      'safe_sampled_action_count':len(safe),
                      'completion_evidence':'SAFE_ACTIONS_AVAILABLE' if safe else
                          ('NO_SAFE_SAMPLED_ACTION' if complete else 'SEARCH_INCOMPLETE'),
                      'all_unknown_unreachable_proven':False,'world_complete':False},
                  'frontier_horizons':{name:[c['frontier_id'] for c in safe if c['target_distance_tier']==name]
                                       for name in ('local','regional','distant')},
                  'current_region_id':region_id(start), 'search_complete':complete,
                  'search_scope':'sampled_current_map_poses_not_all_possible_poses',
                  'planning_attempts':attempted, 'shortlist_size':len(candidates),
                  'candidate_offset':offset,'next_candidate_offset':offset+attempted,'more_candidates':more,
                  'created_unix_s': time.time(), 'expires_unix_s': time.time()+120,
                  'map_version': self.latest['map_version'], 'robot_pose': self.pose(),
                  'sensor_fresh': self.freshness(), 'candidates': safe, 'rejected': rejected,
                  'generation': info, 'status': 'ready' if safe else ('no_safe_reachable_frontiers' if complete else 'candidate_search_incomplete'),
                  'requires_execution_revalidation': True}
        if bridge:
            # Shadow work runs after the baseline planning budget and does not
            # remove candidates or alter scores. Its fields are hidden from GPT.
            if mode=='shadow':
                result['candidates']=[bridge.enrich(c,checked_paths[c['frontier_id']])[0] for c in safe]
            for c,outcome in plan_rejections:bridge.rejected_plan(c,outcome)
            result['exploration_upgrade']=bridge.summary()
            result['exploration_upgrade'].update(candidate_search_scope=result['search_scope'],candidate_search_truncated=not complete)
            result['exploration_upgrade']['candidate_filter_counts']={'pose_proposals':len(candidates),
                'planned':attempted,'offered':len(safe),'planning_rejected':len(plan_rejections),
                'policy_rejected':result['exploration_upgrade']['policy_rejected_count']}
            if mode=='memory' and not safe and complete and result['exploration_upgrade']['policy_rejected_count']:
                result['status']='no_safe_informative_candidate'
            from dataclasses import asdict
            bridge.save(batch,{**info,'pose_evaluations':generation_audit,'settings':asdict(model.settings)},raw_candidates,result)
        self.store.set_meta('catalog', result)
        return result

    def timing_phase(self, phase):
        if hasattr(self, 'phase_timer'): self.phase_timer.mark(phase)

    def stop_confirmed(self, timeout=8.):
        if getattr(self, 'dispatched', False): self.timing_phase('stop_verification')
        return super().stop_confirmed(timeout)

    def perform(self):
        from robot_skills.timing import PhaseTimer
        self.phase_timer=PhaseTimer(lambda: self.node.get_clock().now().nanoseconds*1e-9 or None)
        self.motion_seconds={}
        self.timing_phase('initial_feedback_wait')
        result = {'status': 'rejected', 'reason': None, 'stopped': False}
        try:
            self.store.update(self.identifier, status='running', phase='validating')
            self.ready()
            if self.two_stage:self.require_contract()
            result['before'] = self.sample()
            self.timing_phase('preplan_validation')
            self.metric_before = self.grid_snapshot()
            self.save_map('gain_before')
            if self.task['kind'] in NAVIGATION_KINDS:
                self.perform_investigation_navigation(result)
                return result
            observation = self.task['kind'] == 'perform_observation'
            if observation:
                candidate = self.task['candidate']
                result['candidate'] = candidate
                self.prepare_observation(candidate)
                if candidate['type'] == 'rotate':
                    self.perform_rotation(candidate, result)
                    return result
            if self.fixed:
                x, y, a = self.pose()
                candidate = {'x': x+.4*math.cos(a), 'y': y+.4*math.sin(a), 'yaw': a}
            elif not observation:
                candidate = self.task['candidate']
                catalog = self.store.meta('catalog', {})
                if not any(c['frontier_id'] == candidate['frontier_id'] for c in catalog.get('candidates', [])):
                    raise RuntimeError('frontier_no_longer_exists')
                if time.time() > catalog.get('expires_unix_s', 0):
                    raise RuntimeError('frontier_catalog_expired')
                if self.two_stage:
                    from robot_skills.investigation import repetition_reason
                    why=repetition_reason(self.store.meta('investigation_memory:'+self.task['mission_id'],[]),
                        [candidate],candidate.get('region_epoch'),self.pose())
                    if why:raise RuntimeError(why)
                if not self.model().is_safe(candidate['x'], candidate['y'], candidate['yaw']):
                    if self.task.get('exploration_mode','baseline')!='baseline':
                        from robot_skills.upgrade_geometry import footprint_evidence
                        result['goal_footprint_evidence']=footprint_evidence(self.grid_snapshot(),[candidate['x'],candidate['y'],candidate['yaw']])
                    raise RuntimeError('frontier_pose_no_longer_safe')
            result['candidate'] = candidate
            result['catalog_map_version'] = self.task.get('map_version')
            result['map_changed_since_catalog'] = self.task.get('map_version') != self.latest['map_version']
            self.timing_phase('planning')
            self.store.update(self.identifier, phase='planning')
            plan, outcome = self.plan(candidate)
            result['planning'] = outcome
            self.timing_phase('execution_validation')
            if plan is None:
                raise RuntimeError('path_rejected')
            if plan['length'] > (.8 if self.fixed else (2 if observation else 8)):
                raise RuntimeError('path_budget_exhausted')
            from observation import action_time_estimate,navigation_prediction
            result['revalidated_time_estimate']=action_time_estimate(plan['length'],
                navigation_prediction(plan['path'],self.pose(),[candidate['x'],candidate['y'],candidate['yaw']]),
                self.grid_snapshot()['resolution'])
            self.recheck_exploration_policy(candidate,plan)
            # Validation may be expensive. If refreshing feedback changes the map,
            # repeat the whole check; never send a goal against an older revision.
            for _ in range(3):
                self.health()
                version = self.latest['map_version']
                if not self.fixed and not self.model().is_safe(candidate['x'], candidate['y'], candidate['yaw']):
                    raise RuntimeError('frontier_invalidated_before_dispatch')
                clearance = self.evaluate_path(plan['path'], candidate)
                result['clearance'] = clearance
                if not clearance['safe']:
                    raise RuntimeError('path_invalidated_before_dispatch')
                self.health()
                if self.two_stage and version!=self.latest['map_version']:
                    clearance=self.evaluate_path(plan['path'],candidate)
                    result['clearance']=clearance
                    if not clearance['safe']:raise RuntimeError('path_invalidated_before_dispatch')
                    break
                if version == self.latest['map_version']:
                    self.recheck_exploration_policy(candidate,plan)
                    self.health()
                    if version!=self.latest['map_version']:continue
                    break
            else:
                raise RuntimeError('map_unstable_during_validation')
            if self.task.get('mission_id'):
                from robot_skills import mission
                current_mission=self.store.meta('mission:'+self.task['mission_id'])
                result['dispatch_budget_check']=mission.action_budget(current_mission,self.store.tasks(),
                    self.node.get_clock().now().nanoseconds*1e-9,
                    result['revalidated_time_estimate']['budget_sim_s'],plan['length'])
                if not result['dispatch_budget_check']['fits']:
                    raise RuntimeError('revalidated_action_over_budget')
            goal = NavigateToPose.Goal()
            goal.pose = self.goal_pose(candidate)
            goal.behavior_tree = str(WORKSPACE / 'robot_agent/motion_trial.xml')
            self.active_candidate, self.execution_path = candidate, plan['path']
            self.execution_started_ns = self.node.get_clock().now().nanoseconds
            self.store.update(self.identifier, phase='navigating', map_version=self.latest['map_version'])
            self.timing_phase('navigation_including_turns')
            outcome = self.execute(self.nav, goal, self.args.goal_timeout, 'navigation')
            result['navigation_result'] = outcome
            result['status'] = 'succeeded' if outcome['status'] == 4 else 'aborted'
            result['reason'] = 'goal_reached' if outcome['status'] == 4 else outcome.get('error_msg', 'nav2_aborted')
        except (Exception, KeyboardInterrupt) as error:
            result['reason'] = str(error) or 'interrupted'
            if self.store.should_cancel(self.identifier):
                result['status'] = 'canceled'
            elif self.dispatched:
                result['status'] = 'aborted'
        finally:
            self.timing_phase('stop_verification')
            self.canceling = True
            pending_disabled = True
            if self.dispatched and self.handle is None and self.pending_goal is not None and 'navigation_result' not in result:
                try:
                    handle = self.wait(self.pending_goal, 3)
                    if handle.accepted:
                        self.handle, self.action_result = handle, handle.get_result_async()
                except Exception:
                    result['navigation_paused'] = self.pause_navigation()
                    pending_disabled = result['navigation_paused']
            result['stopped'] = self.cancel_active() and pending_disabled
            if not result['stopped'] and self.dispatched:
                result['navigation_paused'] = self.pause_navigation()
                result['stopped'] = bool(result['navigation_paused'] and self.stop_confirmed())
                if not result['stopped']:
                    result['status'] = 'stop_unconfirmed'
            result['distance_odom_m'] = self.distance
            result['minimum_scan_m'] = self.minimum_scan if math.isfinite(self.minimum_scan) else None
            self.timing_phase('map_feedback_wait')
            if hasattr(self, 'metric_before') and result['stopped']:
                try:
                    result['map_gain'] = self.observation_feedback()
                except Exception as error:
                    result['map_gain'] = {'gain_status':'FEEDBACK_INCOMPLETE', 'usable_for_trend':False,
                                          'reason':'metric_collection_failed:'+type(error).__name__}
                if self.task['kind'] == 'perform_observation':
                    result['observation_metrics'] = result['map_gain']
            try:
                result['after'] = self.sample()
                result['known_area_gain_m2'] = result['after']['known_area_m2'] - result['before']['known_area_m2']
                metrics = result.get('map_gain', {})
                metrics['raw_known_area_delta_m2'] = result['known_area_gain_m2']
                elapsed = result['after']['simulation_seconds']-result['before']['simulation_seconds']
                wall = result['after']['wall_elapsed']-result['before']['wall_elapsed']
                gain = metrics.get('observed_new_known_area_m2') if metrics.get('usable_for_trend') else None
                metrics['comparable_gain_m2']=gain
                metrics.update(distance_odom_m=self.distance, elapsed_sim_s=elapsed, elapsed_wall_s=wall,
                    new_area_per_meter=(gain/self.distance if gain is not None and self.distance>.05 else None),
                    new_area_per_sim_s=(gain/elapsed if gain is not None and elapsed>0 else None),
                    new_area_per_wall_s=(gain/wall if gain is not None and wall>0 else None))
            except (RuntimeError, KeyError):
                pass
            self.timing_phase('terminal_metrics')
            if self.task.get('exploration_mode','baseline')!='baseline':
                try:
                    from robot_skills.upgrade_geometry import as_grid,failure_evidence
                    result['exploration_evidence']=failure_evidence(as_grid(self.grid_snapshot()),self.task['candidate'],result,
                                                                    getattr(self,'last_clearance',None))
                except Exception as error:
                    result['exploration_evidence']={'failure_scope':'system','patch':None,'error':type(error).__name__}
            self.timing_phase(None)
            result['phase_timing']=self.phase_timer.rows
            result['navigation_motion_sim_s']=self.motion_seconds
            result['navigation_motion_scope']='odometry stamped intervals; turn_while_translating overlaps translation; gaps above .5s excluded'
            result['trajectory']=[t['pose'] for t in self.traces if t.get('pose')]
            result['failure_code']=None if result['status']=='succeeded' else failure_code(result['reason'])
            self.report.update(result)
            self.checkpoint()
            self.store.update(self.identifier, status=result['status'], reason=result['reason'],
                              stopped=result['stopped'], sensor_fresh=self.freshness(),
                              map_version=self.latest.get('map_version'), phase='finished', result=result,
                              dispatched=self.dispatched, failure_code=result.get('failure_code'))
        return result

    def recheck_exploration_policy(self,candidate,plan):
        if self.task.get('exploration_mode','baseline')!='memory':return
        epoch=self.store.meta('regional_epoch',{}).get('id')
        if epoch!=candidate.get('region_epoch'):raise RuntimeError('exploration_epoch_changed')
        from observation import navigation_prediction
        bridge=self.upgrade_bridge(self.grid_snapshot(),self.pose(),epoch,self.task['mission_id'],'memory')
        advice=bridge.advise(candidate,navigation_prediction(plan['path'],self.pose(),[candidate['x'],candidate['y'],candidate['yaw']]))
        if not advice['retain_for_safety_recheck']:
            raise RuntimeError('exploration_policy_rejected:'+advice['reason'])


class Stopper:
    def __init__(self):
        self.node = rclpy.create_node('robot_skill_stop')
        self.latest = {}
        def odom(msg):
            self.latest.update(odom_at=time.monotonic(),
                               velocity=[msg.twist.twist.linear.x, msg.twist.twist.angular.z])
        self.subscription = self.node.create_subscription(Odometry, '/model/vehicle/odometry', odom, qos_profile_sensor_data)

    def spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=.05)

    def wait(self, future, timeout):
        deadline = time.monotonic()+timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError('service_timeout')
            self.spin_once()
        return future.result()

    def lifecycle(self, command):
        client = self.node.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        if not client.wait_for_service(timeout_sec=3):
            return False
        request = ManageLifecycleNodes.Request(); request.command = command
        return self.wait(client.call_async(request), 15).success

    def stop(self):
        cancel_return = {}
        for action in ('navigate_to_pose', 'spin'):
            client = self.node.create_client(CancelGoal, '/'+action+'/_action/cancel_goal')
            if client.wait_for_service(timeout_sec=3):
                cancel_return[action] = self.wait(client.call_async(CancelGoal.Request()), 8).return_code
            else:
                cancel_return[action] = None
            self.node.destroy_client(client)
        paused = self.lifecycle(ManageLifecycleNodes.Request.PAUSE)
        stopped = Explorer.stop_confirmed(self, 8)
        return {'navigation_paused': paused, 'cancel_return_code': cancel_return, 'stopped': stopped,
                'status': 'succeeded' if paused and stopped else 'stop_unconfirmed',
                'reason': 'stop_confirmed_and_latched' if paused and stopped else 'stop_unconfirmed'}

    def navigation_active(self):
        for name in ('planner_server','controller_server','behavior_server','bt_navigator'):
            client = self.node.create_client(GetState, '/'+name+'/get_state')
            if not client.wait_for_service(timeout_sec=3):
                return False
            if self.wait(client.call_async(GetState.Request()), 5).current_state.id != 3:
                return False
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('operation', choices=['catalog', 'observations', 'task', 'resume'])
    parser.add_argument('task_id', nargs='?')
    args = parser.parse_args()
    store = Store()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    worker = None
    try:
        if args.operation == 'resume':
            if store.meta('active_mission'):
                raise RuntimeError('mission_still_active')
            generation = store.meta('stop_generation', 0)
            if any(t['status'] in ('accepted','running','stopping') or
                   (t['status'] in ('indeterminate','stop_unconfirmed') and not t.get('resolved')) for t in store.tasks()):
                raise RuntimeError('unresolved_task')
            worker = Stopper()
            if not Explorer.stop_confirmed(worker, 8):
                raise RuntimeError('stop_unconfirmed')
            if not worker.navigation_active() and not worker.lifecycle(ManageLifecycleNodes.Request.RESUME):
                raise RuntimeError('resume_failed')
            try:
                store.clear_stop_after_verified_resume(generation)
            except ValueError:
                worker.lifecycle(ManageLifecycleNodes.Request.PAUSE)
                raise
            result = {'status': 'resumed', 'stop_latched': False}
        elif args.operation == 'task' and store.get(args.task_id)['kind'] == 'stop_robot':
            store.update(args.task_id, status='running', phase='stopping')
            worker = Stopper(); result = worker.stop()
            store.update(args.task_id, **result, phase='finished', result=result,
                         sensor_fresh={'odometry': time.monotonic()-worker.latest.get('odom_at', -math.inf)<1.5})
        else:
            task = store.get(args.task_id) if args.operation == 'task' else None
            worker = Executor(store, task)
            result = ((worker.observation_catalog() if args.operation == 'observations' else worker.catalog()) if task is None
                      else worker.perform_query() if task['kind'] in QUERY_KINDS else worker.perform())
        print(json.dumps(result, allow_nan=False), flush=True)
    except (Exception, KeyboardInterrupt) as error:
        import traceback
        traceback.print_exc()
        if args.operation == 'task':
            if worker is None and isinstance(error, BlockingIOError):
                store.update(args.task_id, status='rejected', reason='another_navigation_owner')
                return
            store.update(args.task_id, status='aborted', reason=str(error) or 'interrupted')
        else:
            print(json.dumps({'status': 'unavailable', 'reason': str(error) or 'interrupted'}), flush=True)
        raise SystemExit(1)
    finally:
        if isinstance(worker, Executor):
            worker.close()
        elif worker:
            worker.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
