"""Compare modes on an archived robot map; never contacts ROS or sends motion."""
import argparse
from contextlib import closing
from dataclasses import asdict
import json
from pathlib import Path
import sqlite3
import sys
import tempfile
import uuid
import numpy as np

WORKSPACE=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(WORKSPACE),str(WORKSPACE/'navigation_base/exploration')]
from aggressive import AggressiveMap
from robot_skills.store import Store
from robot_skills.upgrade_geometry import UpgradeGeometry,as_grid,failure_evidence
from robot_skills.exploration_upgrade import memory_store
from robot_skills.provenance import source_manifest
from exploration_support.memory import Event


def recorded_grid(directory,task):
    path=directory/(task['task_id']+'.detail.gain_settled.npz')
    with np.load(path) as z:
        return {'data':z['data'].copy(),'resolution':float(z['resolution']),'origin':tuple(z['origin']),
                'frame':'map','stamp_sim_s':task['result']['after']['simulation_seconds']}


def replay(directory,task_id):
    uuid.UUID(task_id)
    with closing(sqlite3.connect((directory/'tasks.sqlite3').resolve().as_uri()+'?mode=ro',uri=True)) as db:
        tasks=[json.loads(row[0]) for row in db.execute('SELECT body FROM tasks')]
    task=next(t for t in tasks if t['task_id']==task_id)
    snapshot=recorded_grid(directory,task);pose=task['result']['after']['pose']
    offset=tuple(task['result']['clearance']['sensor_offset_m'])+(0.,)
    epoch=task['candidate']['region_epoch'];mid=task['mission_id']
    past=sorted([t for t in tasks if t['created_unix_s']<=task['created_unix_s']],key=lambda t:t['created_unix_s'])
    excluded=[(t['candidate']['x'],t['candidate']['y']) for t in past if t.get('accepted') and t.get('candidate')]
    with tempfile.TemporaryDirectory(prefix='exploration-replay-') as tmp:
        isolated=Store(tmp);events=[]
        for step,t in enumerate([t for t in past if t.get('accepted') and t.get('mission_id')==mid and t['kind']=='execute_frontier'],1):
            c=t['candidate'];r=t.get('result') or {}
            if c.get('region_epoch')!=epoch:continue
            try:
                grid=as_grid(recorded_grid(directory,t));evidence=failure_evidence(grid,c,r)
            except (FileNotFoundError,KeyError):evidence={'failure_scope':'system','patch':None}
            gain=r.get('map_gain') or {}
            target=r.get('after',{}).get('pose') if t['status']=='succeeded' else None
            event=Event('terminal:'+t['task_id'],mid,epoch,step,tuple(target or [c['x'],c['y'],c['yaw']]),
                        t['status'],t['stopped'],purpose='wait' if t['status']=='succeeded' and target is None else 'transit' if c.get('transit') else 'observe_goal',
                        gain_m2=gain.get('observed_new_known_area_m2') if gain.get('usable_for_trend') and target is not None and t['status']=='succeeded' and t['stopped'] else None,
                        reason=t.get('reason') or '',failure_scope=evidence.get('failure_scope','system'),
                        approach_key=evidence.get('approach_key',''),patch=evidence.get('patch'))
            memory_store(isolated).append(event);events.append(event)
        outputs={}
        model=AggressiveMap(snapshot['data'],snapshot['resolution'],snapshot['origin'],sensor_offset=offset[:2])
        for mode in ('baseline','shadow','memory'):
            audit=[]
            candidates,info=model.candidates(pose,excluded if mode!='memory' else [],regional=True,audit=audit)
            bridge=UpgradeGeometry(isolated,mid,epoch,snapshot,pose,offset,None,mode) if mode!='baseline' else None
            if bridge:bridge.step=len(events)+1
            proposed=[]
            for index,c in enumerate(candidates):
                c={**c,'frontier_id':'OFFLINE_ONLY_'+str(index),'region_epoch':epoch,'transit':False}
                if bridge:
                    row,keep=bridge.enrich(c,[])
                    if mode=='memory' and not keep:continue
                else:row=c
                proposed.append(row)
            outputs[mode]={'generation':info,'pose_proposals':proposed,
                           'diagnostics':bridge.summary() if bridge else None,
                           'all_pose_evaluations':audit,'path_validation':'NOT_PERFORMED'}
        base=outputs['baseline']['pose_proposals'];shadow=outputs['shadow']['pose_proposals']
        same=base==[{k:v for k,v in c.items() if k!='exploration_upgrade'} for c in shadow]
        if not same:raise AssertionError('shadow_changed_baseline_proposals')
        return {'scope':'offline_recorded_map_and_authoritative_task_replay_no_motion',
                'source_task_id':task_id,'source_map_version':task['result']['after']['map_version'],
                'source_map_frame_assumption':'map as recorded in the existing ROS worker',
                'sensor_tf_evidence':'recorded_xy_only; yaw0_assumption_for_full_circle_offline_proxy',
                'sensor_range_evidence':'not_recorded; bounded8m_proxy_not_calibrated_mapping_range',
                'memory_reconstruction':'temporary_database_only; preserves original mission/epoch; original task DB opened read-only',
                'history_events':len(events),'incomparable_gain_events':sum(e.gain_m2 is None for e in events),
                'shadow_preserves_baseline':same,'modes':outputs,'motion_authorized':False,
                'source_sha256':source_manifest()}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task-id',required=True)
    parser.add_argument('--store',type=Path,default=WORKSPACE/'log/robot-skills')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();report=replay(args.store,args.task_id)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'output':str(args.output),'shadow_preserves_baseline':report['shadow_preserves_baseline'],
                     'history_events':report['history_events'],'incomparable_gain_events':report['incomparable_gain_events'],
                     'pose_proposals':{k:len(v['pose_proposals']) for k,v in report['modes'].items()},
                     'motion_authorized':False},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
