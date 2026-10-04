"""ROS-free geometry adapter for the optional exploration modes.

RETAIN is only policy eligibility. No command, map mutation, safety threshold,
recovery, or model-controlled task purpose is implemented here.
"""
from dataclasses import asdict
import json
import math
from pathlib import Path
import numpy as np

from exploration_support.adapter import enrich_existing_candidates
from exploration_support.grid import Grid, capture_patch, patch_change
from exploration_support.memory import Event
from exploration_support.policy import Candidate, Config, filter_for_recheck, low_gain_cycle
from .exploration_upgrade import memory_store, history_revision, reconcile


def as_grid(snapshot):
    return Grid(snapshot['data'],snapshot['resolution'],tuple(snapshot['origin']),snapshot.get('frame','map'))


def witness_key(pose):
    # Equality is not used for geometric matching: nearby samples also match.
    return 'witness-v1:'+json.dumps([round(pose[0]*2)/2,round(pose[1]*2)/2,
                                    round(math.remainder(pose[2],2*math.pi)/(math.pi/4))%8],separators=(',',':'))


def path_matches(key,path):
    if not key.startswith('witness-v1:'): return False
    x,y,b=json.loads(key.split(':',1)[1]);angle=b*math.pi/4
    # Include in-place turns as well as forward segment directions. This is a
    # local witness matcher, not a topological proof of corridor identity.
    samples=[]
    for a,c in zip(path,path[1:]):
        if math.dist(a[:2],c[:2])>1e-6:
            heading=math.atan2(c[1]-a[1],c[0]-a[0])
            count=max(1,math.ceil(math.dist(a[:2],c[:2])/.25))
            samples.extend([a[0]+(c[0]-a[0])*i/count,a[1]+(c[1]-a[1])*i/count,heading] for i in range(count+1))
        else:
            turn=math.remainder(c[2]-a[2],2*math.pi)
            count=max(1,math.ceil(abs(turn)/(math.pi/8)))
            samples.extend([a[0],a[1],a[2]+turn*i/count] for i in range(count+1))
    return any(math.hypot(p[0]-x,p[1]-y)<=.75 and abs(math.remainder(p[2]-angle,2*math.pi))<=math.pi/4 for p in samples)


def failure_evidence(grid,candidate,result,clearance=None):
    """Only witnessed geometry creates a spatial policy restriction."""
    if result.get('status')=='succeeded':
        pose=result.get('after',{}).get('pose')
        return {'failure_scope':'none','patch':capture_patch(grid,pose[:2],8.,.25) if pose else None}
    check=clearance or result.get('clearance') or (result.get('planning') or {}).get('clearance') or {}
    if check.get('safe') is False and check.get('reason')=='path_guard_clearance' and check.get('worst_base_pose'):
        pose=check['worst_base_pose'];center=check.get('worst_sensor_xy',pose[:2])
        return {'failure_scope':'approach','approach_key':witness_key(pose),
                'failure_witness_map_xy':center,'reason_code':'PATH_GUARD_CLEARANCE',
                'patch':capture_patch(grid,center,3.5,.25)}
    footprint=result.get('goal_footprint_evidence',{})
    if footprint.get('reason_code') in ('GOAL_FOOTPRINT_UNKNOWN','GOAL_FOOTPRINT_OCCUPIED','GOAL_FOOTPRINT_UNCERTAIN'):
        return {'failure_scope':'goal_pose','reason_code':footprint['reason_code'],
                'patch':capture_patch(grid,(candidate['x'],candidate['y']),3.5,.25)}
    reason=result.get('reason') or ''
    code=('PATH_BUDGET_EXCEEDED' if reason=='path_budget_exhausted' else
          'LOCALIZATION_STALE' if 'localization' in reason else
          'SENSOR_STALE' if 'stale' in reason or 'scan_invalid' in reason else
          'STOP_LATCHED' if reason in ('stop_requested','stop_latched') else
          'INDETERMINATE' if result.get('status') in ('indeterminate','stop_unconfirmed') else
          'GOAL_TIMEOUT' if reason=='goal_timeout' else
          'NO_NAV2_PATH' if result.get('planning') and result['planning'].get('status')!=4 else
          'NON_SPATIAL_FAILURE')
    return {'failure_scope':'system','patch':None,'reason_code':code}


def footprint_evidence(snapshot,pose):
    from observation import navigation_footprint
    body,padding=navigation_footprint();r=snapshot['resolution'];data=snapshot['data']
    ox,oy,oa=snapshot['origin'];c,s=math.cos(oa),math.sin(oa)
    center=np.array([c*(pose[0]-ox)+s*(pose[1]-oy),-s*(pose[0]-ox)+c*(pose[1]-oy)])
    angle=pose[2]-oa;c,s=math.cos(angle),math.sin(angle)
    polygon=body@np.array([[c,s],[-s,c]])+center
    margin=padding+r*math.sqrt(2)+.025
    low=np.floor((polygon.min(axis=0)-margin)/r).astype(int);high=np.ceil((polygon.max(axis=0)+margin)/r).astype(int)
    xx,yy=np.meshgrid(np.arange(low[0],high[0]+1),np.arange(low[1],high[1]+1))
    points=np.column_stack(((xx.ravel()+.5)*r,(yy.ravel()+.5)*r));inside=np.ones(len(points),dtype=bool)
    for p,q in zip(polygon,np.roll(polygon,-1,axis=0)):
        edge=q-p;inside&=(edge[0]*(points[:,1]-p[1])-edge[1]*(points[:,0]-p[0]))>=-margin*np.linalg.norm(edge)
    x,y=xx.ravel()[inside],yy.ravel()[inside]
    valid=(x>=0)&(y>=0)&(x<data.shape[1])&(y<data.shape[0]);cells=data[y[valid],x[valid]]
    counts={'unknown':int((cells<0).sum()),'occupied':int((cells>=65).sum()),
            'uncertain':int(((cells>=25)&(cells<65)).sum()),'outside_map':int((~valid).sum())}
    reason=('GOAL_FOOTPRINT_OCCUPIED' if counts['occupied'] else 'GOAL_FOOTPRINT_UNKNOWN' if counts['unknown'] or counts['outside_map'] else
            'GOAL_FOOTPRINT_UNCERTAIN' if counts['uncertain'] else 'GOAL_PREFILTER_OR_OTHER_CONSTRAINT')
    return {'raw_cells_under_padded_footprint':counts,'reason_code':reason,'is_safety_certificate':False}


class UpgradeGeometry:
    def __init__(self,store,mission_id,epoch,snapshot,robot_pose,offset,scan_range,mode):
        if mode not in ('shadow','memory'): raise ValueError('invalid_upgrade_mode')
        self.store,self.mid,self.epoch,self.mode=store,mission_id,epoch,mode
        self.grid=as_grid(snapshot);self.snapshot=snapshot;self.robot=robot_pose;self.offset=tuple(offset)
        self.range=min(8.,float(scan_range)) if scan_range is not None and math.isfinite(scan_range) and scan_range>0 else 8.
        self.range_source='bounded_8m_proxy_capped_by_scan' if scan_range else 'bounded_8m_proxy_sensor_limit_unavailable'
        reconcile(store,mission_id)
        self.events=memory_store(store).events(mission_id,epoch)
        self.revision=history_revision(store.tasks(),mission_id)
        self.step=1+sum(t.get('mission_id')==mission_id and t['kind']!='stop_robot' for t in store.tasks())
        self.failures=[Event(**{**e,'target':tuple(e['target'])}) for e in store.meta('exploration_planning:'+mission_id,[])
                       if e['map_epoch']==epoch]
        self.audit=[];self.plans=[]

    def advise(self,candidate,path):
        purpose='transit' if candidate.get('transit') else 'observe_goal'
        c=Candidate(candidate.get('frontier_id',candidate.get('id','candidate')),candidate['x'],candidate['y'],candidate['yaw'],
                    max(0,candidate.get('estimated_gain_m2',0)),max(0,candidate.get('planned_length_m',0)),purpose=purpose)
        answer=filter_for_recheck(self.grid,[c],self.events,self.mid,self.epoch,self.step)[0]
        # Match each prior failed approach geometrically against this newly
        # planned path; a different entry to the same goal remains eligible.
        for event in self.events+self.failures:
            if event.failure_scope=='approach' and path_matches(event.approach_key,path):
                scoped=Candidate(**{**asdict(c),'approach_key':event.approach_key})
                check=filter_for_recheck(self.grid,[scoped],[event],self.mid,self.epoch,self.step)[0]
                if not check.retain_for_safety_recheck: answer=check;break
        return asdict(answer)

    def enrich(self,candidate,path):
        row=dict(candidate)
        try:
            extra=enrich_existing_candidates(self.grid,tuple(self.robot),[candidate],self.offset,self.range,180)[0]
            proxy={k:v for k,v in extra.items() if k not in candidate}
        except ValueError as error:
            proxy={'status':'visibility_unavailable','reason':str(error),'visibility_is_safety_certificate':False}
        advice=self.advise(candidate,path)
        audit={'candidate_id':candidate.get('frontier_id',candidate.get('id')),
               'mode':self.mode,'visibility':proxy,'policy':advice,'purpose':'transit' if candidate.get('transit') else 'observe_goal'}
        row['exploration_upgrade']=audit
        self.audit.append(audit);self.plans.append({'candidate':row,'checked_path':path})
        return row,advice['retain_for_safety_recheck']

    def rejected_plan(self,candidate,outcome):
        evidence=failure_evidence(self.grid,candidate,{'status':'rejected','planning':outcome})
        self.plans.append({'candidate':candidate,'planning_rejection':outcome,'evidence':evidence})
        if evidence['failure_scope']!='approach':return
        key=evidence['approach_key'];patch=evidence['patch']
        event=Event('preflight:'+key+':'+patch['digest'],self.mid,self.epoch,self.step,
                    (candidate['x'],candidate['y'],candidate['yaw']),'rejected',False,purpose='wait',
                    reason='PATH_GUARD_CLEARANCE',failure_scope='approach',approach_key=key,patch=patch)
        if any(e.event_id==event.event_id for e in self.failures):return
        self.failures=(self.failures+[event])[-128:]
        # Diagnostic rejection memory is separate from real terminal events;
        # it never increments task/visit counts or forms an ABAB observation.
        self.store.set_meta('exploration_planning:'+self.mid,[asdict(e) for e in self.failures])

    def summary(self):
        cycle=low_gain_cycle(self.events,Config())
        unchanged_cycle=bool(cycle) and all(e.patch is not None and not patch_change(self.grid,e.patch)[0] for e in cycle)
        return {'mode':self.mode,'mission_id':self.mid,'map_epoch':self.epoch,
                'history_revision':self.revision,'events':len(self.events),
                'low_gain_pattern_detected':bool(cycle),'low_gain_cycle_detected':unchanged_cycle,
                'decision_mode':'LOCAL_OR_EXPAND',
                'policy_rejected_count':sum(not a['policy']['retain_for_safety_recheck'] for a in self.audit),
                'candidate_advice':self.audit,'range_source':self.range_source,
                'scan_statistics':getattr(self,'scan_statistics',{}),
                'remaining_unknown_is_not_completion_proof':True,'is_motion_permission':False,
                'recent_action_summaries':[{k:v for k,v in asdict(e).items() if k!='patch'} for e in self.events[-6:]]}

    def save(self,catalog_id,generation,raw_candidates,result):
        directory=self.store.directory/'exploration-upgrade';directory.mkdir(exist_ok=True)
        path=directory/catalog_id
        np.savez_compressed(str(path)+'.map.npz',data=self.grid.data,resolution=self.grid.resolution,origin=self.grid.origin)
        payload={'raw_map_frame':self.grid.frame,'map_metadata':{k:v for k,v in self.snapshot.items() if k!='data'},
                 'robot_pose':self.robot,'base_to_lidar_tf':self.offset,'range_m':self.range,
                 'generation':generation,'raw_candidates':raw_candidates,'plans_and_advice':self.plans,
                 'catalog':result,'motion_commands_sent':False}
        from robot_skills.provenance import source_manifest
        payload['source_sha256']=source_manifest()
        Path(str(path)+'.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
