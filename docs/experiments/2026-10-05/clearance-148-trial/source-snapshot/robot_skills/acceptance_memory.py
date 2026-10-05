"""Three targeted offline cases with isolated synthetic authoritative history.

This does not inject observations into the live ledger, command a robot, or
claim that fabricated gains were measured during simulation.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import tempfile
import numpy as np

WORKSPACE=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(WORKSPACE),str(WORKSPACE/'navigation_base/exploration'),str(WORKSPACE/'navigation_base')]
from robot_skills.store import Store
from robot_skills.upgrade_geometry import UpgradeGeometry,witness_key
from exploration_support.memory import Event
from exploration_support.grid import Grid,capture_patch
from path_safety import PathSafety


def acceptance():
    data=np.zeros((240,240),dtype=np.int16);data[:,220:]=-1
    origin=(-10.,-10.,0.);grid=Grid(data,.25,origin)
    snapshot={'data':data,'resolution':.25,'origin':origin,'frame':'map','stamp_sim_s':1.}
    with tempfile.TemporaryDirectory() as directory:
        bridge=UpgradeGeometry(Store(directory),'fixture-mission','fixture-epoch',snapshot,[3,4,0],(0,0,0),8,'memory')
        bridge.step=5
        def event(step,pose,gain=.02):
            return Event('fixture:'+str(step),'fixture-mission','fixture-epoch',step,tuple(pose),'succeeded',True,
                         gain_m2=gain,patch=capture_patch(grid,pose[:2],8,.25))
        def candidate(x,y,transit=False):
            return {'frontier_id':'refreshed-id','x':x,'y':y,'yaw':0,'estimated_gain_m2':1,
                    'planned_length_m':5,'transit':transit}
        bridge.events=[event(i,p) for i,p in enumerate([(8,4,0),(12,4,0)]*2,1)]
        repeat=bridge.advise(candidate(8,4),[[12,4,0],[8,4,0]])
        assert bridge.summary()['low_gain_cycle_detected'] and not repeat['retain_for_safety_recheck']
        bridge.events=[event(1,(8,4,0),None)]
        incomparable=bridge.advise(candidate(8,4),[[3,4,0],[8,4,0]])
        assert incomparable['retain_for_safety_recheck']
        bridge.events=[event(1,(8,4,0))]
        through=[[3,4,0],[8,4,0],[15,4,0]]
        via=bridge.advise(candidate(15,4),through)
        transit=bridge.advise(candidate(8,4,True),through[:2])
        through_safe=PathSafety(data,.25,origin,'fixture').evaluate(through,through[0],through[-1],(0,0))
        assert via['retain_for_safety_recheck'] and transit['retain_for_safety_recheck'] and through_safe['safe']
        blocked=data.copy();blocked[62,60]=100 # World (5.125,5.625), only the direct entrance.
        blocked_grid=Grid(blocked,.25,origin);bridge.grid=blocked_grid
        bridge.events=[Event('fixture:failed-entry','fixture-mission','fixture-epoch',1,(8,4,0),'aborted',True,
            failure_scope='approach',approach_key=witness_key([5,4,0]),patch=capture_patch(blocked_grid,(5,4),3.5,.25))]
        direct=[[3,4,0],[8,4,0]]
        detour=[[3,4,0],[0,4,0],[0,10,0],[10,10,0],[10,4,0],[8,4,0]]
        same=bridge.advise(candidate(8,4),direct);other=bridge.advise(candidate(8,4),detour)
        model=PathSafety(blocked,.25,origin,'fixture')
        direct_safety=model.evaluate(direct,direct[0],direct[-1],(0,0))
        detour_safety=model.evaluate(detour,detour[0],detour[-1],(0,0))
        assert not same['retain_for_safety_recheck'] and not direct_safety['safe']
        assert other['retain_for_safety_recheck'] and detour_safety['safe']
        return {'scope':'offline synthetic grid and isolated fixture history; no live task injection; no motion',
                'guard_threshold_m':1.9,'required_path_clearance_m':2.15,
                'cases':{'low_gain_ABAB':{'repeat':repeat,'pass':True},
                    'another_entry':{'same_entry':same,'other_entry':other,'direct_safety':direct_safety,'detour_safety':detour_safety,'pass':True},
                    'old_position_to_new_area':{'new_goal_via_old_position':via,'transit_at_old_position':transit,'path_safety':through_safe,'pass':True},
                    'unregistered_gain_is_null':{'advice':incomparable,'gain_m2':None,'pass':True}},
                'online_low_gain_trigger_verified':False}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=acceptance();args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v['pass'] for k,v in result['cases'].items()}))
