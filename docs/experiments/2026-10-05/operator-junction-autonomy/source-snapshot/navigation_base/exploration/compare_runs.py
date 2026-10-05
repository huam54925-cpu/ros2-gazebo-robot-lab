#!/usr/bin/env python3
"""Compare recorded policy runs offline, without supplying maps to either policy."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def load(path):
    r=json.loads(path.read_text())
    trace=json.loads(path.with_suffix('.trace.json').read_text())
    samples=[r['initial']]+trace+[r['final']]
    t0=r['initial']['simulation_seconds']
    samples=sorted(samples,key=lambda s:s['wall_elapsed'])
    return r,samples,np.array([s['simulation_seconds']-t0 for s in samples])


def main():
    p=argparse.ArgumentParser();p.add_argument('baseline',type=Path);p.add_argument('aggressive',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();runs=[load(a.baseline),load(a.aggressive)]
    horizon=min(t[-1] for _,_,t in runs)
    fig,axes=plt.subplots(1,2,figsize=(13,5))
    stats=[]
    for label,color,path,(r,samples,times) in zip(['Conservative','Aggressive'],['#4979ae','#e27625'],[a.baseline,a.aggressive],runs):
        area=np.array([s['known_area_m2'] for s in samples])
        axes[0].plot(times,area,label=label,color=color)
        axes[1].plot([s['path_length_odom_m'] for s in samples],area,label=label,color=color)
        # Last sample at/before common simulated duration: no extrapolated growth.
        index=np.flatnonzero(times<=horizon)[-1]
        after_scan=r['goals'][0]['before']['known_area_m2'] if r['goals'] else r['final']['known_area_m2']
        stats.append({'policy':label,'report':path.name,'goals':r['goal_count'],'successes':r['successful_goals'],
            'failures':r['failed_goals'],'stop_reason':r['stop_reason'],'stopped':r['stopped'],
            'wall_seconds':r['elapsed_wall_seconds'],'simulation_seconds':float(times[-1]),
            'distance_odom_m':r['path_length_odom_m'],'known_area_initial_m2':r['initial']['known_area_m2'],
            'known_area_after_initial_scan_m2':after_scan,'known_area_final_m2':r['final']['known_area_m2'],
            'navigation_phase_gain_m2':r['final']['known_area_m2']-after_scan,
            'navigation_gain_per_meter':(r['final']['known_area_m2']-after_scan)/max(r['path_length_odom_m'],1e-9),
            'minimum_scan_m':r['minimum_scan_m'],'common_time_last_sample_s':float(times[index]),
            'known_area_at_common_time_m2':float(area[index])})
    axes[0].axvline(horizon,color='gray',linestyle=':',label=f'Common horizon: {horizon:.1f} s')
    axes[0].set_xlabel('Simulated elapsed time (s)');axes[1].set_xlabel('Odometry distance (m)')
    for ax in axes:
        ax.set_ylabel('Known-cell area (m²)');ax.grid(alpha=.25);ax.legend()
    fig.suptitle('Frontier policy comparison — single run per policy, not coverage')
    fig.tight_layout();fig.savefig(a.output.with_suffix('.png'),dpi=150)
    a.output.with_suffix('.json').write_text(json.dumps({'common_simulation_horizon_s':float(horizon),'runs':stats,
        'limitations':['One run per policy; no statistical significance claim.','Aggressive changes scoring, candidate geometry and revisit rules together.','Known area is not coverage; initial scan growth is separated from navigation gain.','Goal/time caps match, actual stopping times differ.']},indent=2)+'\n')

if __name__=='__main__':main()
