"""ROS-side map/query and persistent waypoint execution using the shared guard."""
import math
import time
import uuid

from robot_skills.investigation import QUERY_KINDS, repetition_reason, failure_code
from navigation_base.robot_contract import CONTRACT, CONTRACT_HASH


class InvestigationMixin:
    def ensure_epoch(self):
        grid=self.grid_snapshot()
        signature=[grid['frame'],grid['resolution'],grid['origin'][2]]
        memory=self.store.meta('regional_epoch',{})
        if memory.get('geometry')!=signature or grid['stamp_sim_s']<memory.get('last_sim_s',0):
            memory={'id':uuid.uuid4().hex[:12],'geometry':signature}
        memory['last_sim_s']=grid['stamp_sim_s']
        self.store.set_meta('regional_epoch',memory)
        return memory['id']

    def map_snapshot(self):
        from robot_skills.map_view import render
        epoch=self.ensure_epoch()
        traces=[]
        for task in reversed(self.store.tasks()):
            if task.get('mission_id')==(self.task or {}).get('mission_id'):
                result=task.get('result') or {}
                traces.extend(result.get('trajectory',[]))
        traces.extend(t['pose'] for t in self.traces if t.get('pose'))
        snapshot=render(self.grid_snapshot(),self.pose(),epoch,self.latest['map_version'],traces,CONTRACT_HASH)
        snapshot.update(stopped=self.stop_confirmed(),created_unix_s=time.time(),sensor_fresh=self.freshness())
        self.store.set_meta('map_snapshot',snapshot)
        self.save_map('snapshot')
        return snapshot

    def require_contract(self):
        """Detect old guards and inconsistent sensor TF; never activate a profile."""
        from safety_profile import PROFILE, CONTRACT_HASH as active_hash
        if PROFILE!='footprint_075' or active_hash!=CONTRACT_HASH:
            raise RuntimeError('config_contract_mismatch')
        from explore import rclpy,yaw
        t=self.buffer.lookup_transform(CONTRACT['base_frame'],CONTRACT['scan_frame'],rclpy.time.Time())
        actual=[t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)]
        if any(abs(a-b)>.001 for a,b in zip(actual,CONTRACT['scan_in_base'])):
            raise RuntimeError('sensor_transform_contract_mismatch')
        import json
        from rcl_interfaces.srv import GetParameters
        def parameters(node_name,names):
            client=self.node.create_client(GetParameters,node_name+'/get_parameters')
            try:
                if not client.wait_for_service(timeout_sec=3):
                    raise RuntimeError('navigation_contract_parameters_unavailable')
                req=GetParameters.Request();req.names=names
                return self.wait(client.call_async(req),5).values
            finally:
                self.node.destroy_client(client)
        for node in ('/local_costmap/local_costmap','/global_costmap/global_costmap'):
            values=parameters(node,['footprint','footprint_padding','robot_base_frame','track_unknown_space'])
            if (len(values)!=4 or json.loads(values[0].string_value)!=CONTRACT['footprint']
                    or abs(values[1].double_value-CONTRACT['body_padding_m'])>1e-9
                    or values[2].string_value!=CONTRACT['base_frame'] or not values[3].bool_value):
                raise RuntimeError('navigation_footprint_contract_mismatch')
        values=parameters('/controller_server',['FollowPath.max_linear_vel','FollowPath.max_angular_vel',
                                               'FollowPath.use_collision_detection'])
        limits=CONTRACT['profiles']['footprint_075']
        if (len(values)!=3 or not 0<values[0].double_value<=limits['max_linear_m_s']
                or not 0<values[1].double_value<=limits['max_angular_rad_s'] or not values[2].bool_value):
            raise RuntimeError('controller_contract_mismatch')
        publishers=self.node.get_publishers_info_by_topic('/model/vehicle/cmd_vel_safe')
        if len(publishers)!=1 or publishers[0].node_name!='velocity_guard':
            raise RuntimeError('velocity_protection_chain_mismatch')

    def perform_query(self):
        result={'status':'rejected','stopped':False}
        try:
            self.store.update(self.identifier,status='running',phase='query')
            self.ready();self.require_contract()
            if self.task['kind']=='view_map':
                result.update(status='succeeded',reason='map_snapshot',snapshot=self.map_snapshot())
            elif self.task['kind']=='search_frontiers':
                result.update(status='succeeded',reason='search_finished',catalog=self.catalog())
            else:
                if self.ensure_epoch()!=self.task['payload']['map_epoch']:
                    raise RuntimeError('map_epoch_changed')
                # Alternatives from CURRENT pose, not a claim that the whole waypoint chain is valid.
                results=[]
                for pose in self.task['payload']['poses']:
                    self.health()
                    plan,outcome=self.plan(pose)
                    results.append({'goal':pose,'path':plan,'planning':outcome,
                                    'failure_code':None if plan else failure_code((outcome.get('clearance') or {}).get('reason','no_path'))})
                result.update(status='succeeded',reason='planning_query_finished',plans=results,
                              semantics='each_goal_planned_from_current_pose; execution_replans_each_leg')
            result['stopped']=self.stop_confirmed()
        except Exception as error:
            result.update(reason=str(error),failure_code=failure_code(str(error)))
        finally:
            self.store.update(self.identifier,**{k:result[k] for k in ('status','stopped','reason')},
                              phase='finished',result=result,sensor_fresh=self.freshness())
        return result

    def perform_investigation_navigation(self,result):
        from explore import NavigateToPose
        from observation import action_time_estimate,navigation_prediction
        from robot_skills import mission
        self.require_contract()
        payload=self.task['payload'];epoch=self.ensure_epoch()
        if epoch!=payload['map_epoch']:
            raise RuntimeError('map_epoch_changed')
        inv=self.store.meta('investigation:'+payload['investigation_id'])
        if not inv or inv['status']!='active' or inv['map_epoch']!=epoch:
            raise RuntimeError('investigation_no_longer_active')
        why=repetition_reason(self.store.meta('investigation_memory:'+self.task['mission_id'],[]),
                              payload['poses'],epoch,self.pose())
        if why:raise RuntimeError(why)
        result.update(waypoints_reached=0,legs=[],execution_mode='sequential_nav2_poses_with_stop_and_revalidation',
                      contract_hash=CONTRACT_HASH,passage_crossed=False)
        from pathlib import Path
        tree=str(Path(__file__).resolve().parents[1]/'robot_agent/motion_trial.xml')
        for index,candidate in enumerate(payload['poses']):
            self.health()
            if self.ensure_epoch()!=epoch:raise RuntimeError('map_epoch_changed')
            candidate={**candidate,'region_epoch':epoch}
            self.store.update(self.identifier,phase='planning',waypoint_index=index,
                              waypoint_count=len(payload['poses']),candidate=candidate)
            self.timing_phase('planning')
            plan,outcome=self.plan(candidate)
            leg={'goal':candidate,'planning':outcome};result['legs'].append(leg)
            if plan is None:
                raise RuntimeError((outcome.get('clearance') or {}).get('reason','path_rejected'))
            # New content is rechecked; a change in map timestamp alone is not a refusal.
            self.health()
            check=self.evaluate_path(plan['path'],candidate)
            if not check['safe']:raise RuntimeError(check['reason'])
            estimate=action_time_estimate(plan['length'],navigation_prediction(plan['path'],self.pose(),
                [candidate['x'],candidate['y'],candidate['yaw']]),self.grid_snapshot()['resolution'])
            m=self.store.meta('mission:'+self.task['mission_id'])
            leg['budget']=mission.action_budget(m,self.store.tasks(),self.node.get_clock().now().nanoseconds*1e-9,
                                               estimate['budget_sim_s'],plan['length'])
            if not leg['budget']['fits']:raise RuntimeError('revalidated_action_over_budget')
            self.active_candidate,self.execution_path=candidate,plan['path']
            self.execution_started_ns=self.node.get_clock().now().nanoseconds
            goal=NavigateToPose.Goal();goal.pose=self.goal_pose(candidate);goal.behavior_tree=tree
            self.store.update(self.identifier,phase='navigating',dispatched=True)
            self.timing_phase('navigation_including_turns')
            result.pop('navigation_result',None)
            outcome=self.execute(self.nav,goal,self.args.goal_timeout,'navigation')
            result['navigation_result']=outcome;leg['navigation_result']=outcome
            if outcome['status']!=4:
                result.update(status='aborted',reason=outcome.get('error_msg') or 'nav2_aborted')
                return
            result['waypoints_reached']+=1
            self.store.update(self.identifier,waypoints_reached=result['waypoints_reached'])
        result.update(status='succeeded',reason='waypoints_reached')
        # Estimated-pose evidence only; independent simulation truth remains the final evaluator.
        if inv['type']=='verify_passage':
            poses=[result['before']['pose']]+[t['pose'] for t in self.traces if t.get('pose')]+[self.pose()]
            result['passage_crossed']=passage_crossed(poses,inv['passage'])


def passage_crossed(poses,passage):
    """Both full-body sides plus an entry-segment intersection, not mere goal arrival."""
    import numpy as np
    a,b=np.asarray(passage['a']),np.asarray(passage['b'])
    edge=b-a;length=float(np.linalg.norm(edge));normal=np.array([-edge[1],edge[0]])/length
    normal*=passage['destination_side']
    body=np.asarray(CONTRACT['footprint']);padding=CONTRACT['body_padding_m']
    def span(p):
        c,s=math.cos(p[2]),math.sin(p[2]);poly=body@np.array([[c,s],[-s,c]])+p[:2]
        return (poly-a)@normal
    if len(poses)<2 or max(span(poses[0]))>=-padding or min(span(poses[-1]))<=padding:
        return False
    for p,q in zip(poses,poses[1:]):
        p,q=np.asarray(p[:2]),np.asarray(q[:2]);sp=float((p-a)@normal);sq=float((q-a)@normal)
        if sp<0<=sq and sq-sp>1e-9:
            point=p+(-sp/(sq-sp))*(q-p)
            along=float((point-a)@edge)/(length*length)
            if 0<=along<=1:return True
    return False
