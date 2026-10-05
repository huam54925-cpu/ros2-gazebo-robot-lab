"""Read-only candidate comparison on one archived raw map and robot pose.

No ROS, task DB, navigation commands, map writes, or live setting changes.
Broad search uses exactly the existing free/body/obstacle safety prefilters.
"""
import argparse
from dataclasses import replace,asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import numpy as np

WORKSPACE=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(WORKSPACE),str(WORKSPACE/'navigation_base/exploration')]
from aggressive import AggressiveMap
from frontier import components
from exploration_support.grid import Grid,sensor_pose,ray_cells,Visibility,Cell
from exploration_support.frontier_identity import associate,candidate_identity


def visible_unknown_boundary(grid,sensor,range_m=8.,rays=180,labels=None):
    """Same supercover rays/stops as the reference, with cached Python labels.

    This offline implementation avoids three NumPy scalar/Enum comparisons per
    free ray cell. Equivalence is checked against the authoritative reference.
    """
    if rays<4 or not math.isfinite(range_m) or range_m<=0:raise ValueError('invalid_ray_settings')
    x,y,yaw=sensor
    if grid.label_at(x,y)!=Cell.FREE:raise ValueError('sensor_not_in_known_free')
    labels=grid.labels().tolist() if labels is None else labels
    h,w=grid.data.shape;seen=set();occupied=uncertain=outside=reached=0
    for angle in np.linspace(yaw-math.pi,yaw+math.pi,rays,endpoint=False):
        for r,c in ray_cells(grid,x,y,float(angle),range_m):
            if not (0<=r<h and 0<=c<w):outside+=1;break
            label=labels[r][c]
            if label==1:continue
            if label==2:occupied+=1;break
            if label==3:uncertain+=1;break
            if label==0:seen.add((r,c));break
        else:reached+=1
    return Visibility(frozenset(seen),len(seen)*grid.resolution**2,occupied,uncertain,outside,reached)


def visible_search(model,robot,offset,range_m=8.,rays=180,limit=24):
    grid=Grid(model.raw,model.raw_resolution,tuple(model.origin))
    labels=grid.labels().tolist()
    current=visible_unknown_boundary(grid,sensor_pose(tuple(robot),tuple(offset)),range_m,rays,labels).first_unknown_cells
    # A one-coarse-cell neighbourhood discounts ray-lattice jitter. This is
    # information scoring only; no occupancy or collision geometry is changed.
    radius=math.ceil(model.resolution/grid.resolution)
    near={(r+dy,c+dx) for r,c in current for dy in range(-radius,radius+1)
          for dx in range(-radius,radius+1) if math.hypot(dy,dx)*grid.resolution<=model.resolution+1e-8}
    groups=[model.world(g) for g in components(model.frontier)
            if len(g)*model.resolution>=model.settings.minimum_frontier_length]
    if not groups:return [],{'reason':'no_frontier_clusters','all_candidates':[]}
    boundary=np.concatenate(groups);identities=associate(groups)
    points=model.world(np.argwhere(model.safe));selected=[];audit=[]
    counts=dict(safe_grid_cells=len(points),too_near=0,outside_sensor_search_horizon=0,
                sampled_position_duplicate=0,no_safe_heading=0,no_incremental_boundary=0,
                qualified_viewpoints=0)
    checked_positions=[]
    for point in points:
        delta=point-np.asarray(robot[:2]);distance=float(np.linalg.norm(delta))
        if distance<model.settings.minimum_goal_distance:
            counts['too_near']+=1;continue
        distances=np.linalg.norm(boundary-point,axis=1);nearest=float(distances.min())
        # Search radius is sensor range plus actual base-to-lidar offset, not a
        # relaxed motion clearance or an invented lidar range.
        if nearest>range_m+math.hypot(*offset[:2])+model.resolution:
            counts['outside_sensor_search_horizon']+=1;continue
        if any(math.dist(point,p)<.8 for p in checked_positions):
            counts['sampled_position_duplicate']+=1;continue
        target=boundary[np.argmin(distances)];bearing=math.atan2(target[1]-point[1],target[0]-point[0])
        headings=[bearing,bearing+math.pi/2,bearing-math.pi/2,robot[2]]
        headings=[a for a in headings if model.is_safe(*point,a)]
        if not headings:
            counts['no_safe_heading']+=1;audit.append({'xy':point.tolist(),'reason':'NO_SAFE_FOOTPRINT_HEADING'});continue
        rows=[]
        for heading in headings:
            pose=(*point,float(heading));view=visible_unknown_boundary(grid,sensor_pose(pose,tuple(offset)),range_m,rays,labels)
            novel=view.first_unknown_cells-current;robust=view.first_unknown_cells-near
            rows.append({'x':float(point[0]),'y':float(point[1]),'yaw':float(heading),'dx':float(delta[0]),'dy':float(delta[1]),
                'euclidean_distance_m':distance,'frontier_distance_m':nearest,
                'visible_boundary_cells':len(view.first_unknown_cells),'incremental_boundary_cells':len(novel),
                'robust_incremental_boundary_cells':len(robust),'novelty_ratio':len(novel)/len(view.first_unknown_cells) if view.first_unknown_cells else None,
                'robust_boundary_proxy_m2':len(robust)*grid.resolution**2,
                'visible_boundary_world_xy':[grid.world(r,c) for r,c in sorted(view.first_unknown_cells)],
                'robust_novel_boundary_world_xy':[grid.world(r,c) for r,c in sorted(robust)],
                'ray_stops':{'occupied':view.rays_hit_occupied,'uncertain':view.rays_hit_uncertain,
                             'outside_map':view.rays_leave_map,'range_limit':view.rays_reach_range}})
        best=max(rows,key=lambda c:(c['robust_incremental_boundary_cells'],c['incremental_boundary_cells'],c['visible_boundary_cells']))
        if best['robust_incremental_boundary_cells']==0:
            counts['no_incremental_boundary']+=1;audit.append({'xy':point.tolist(),'reason':'NO_ROBUST_INCREMENTAL_BOUNDARY',
                'heading_visibility':[{k:v for k,v in c.items() if not k.endswith('_world_xy')} for c in rows]});continue
        checked_positions.append(point.tolist())
        best.update(candidate_identity(best,identities))
        best['preplan_score']=best['robust_incremental_boundary_cells']/(1.+.12*distance)
        best['id']='READONLY_VISIBLE_'+str(len(selected)+1)
        best['goal_body_prefilter_passed']=True;best['path_validation']='NOT_YET_PERFORMED'
        selected.append(best)
    selected.sort(key=lambda c:c['preplan_score'],reverse=True)
    # Per-cluster round-robin, then fill by score. Do not let a single boundary
    # consume all planner checks before alternative approaches are examined.
    shortlisted=[];buckets={}
    for c in selected:buckets.setdefault(c['frontier_cluster_id'],[]).append(c)
    while any(buckets.values()) and len(shortlisted)<limit:
        for bucket in buckets.values():
            if bucket and len(shortlisted)<limit:shortlisted.append(bucket.pop(0))
    counts['qualified_viewpoints']=len(selected)
    return shortlisted,{'filter_counts':counts,'range_m':range_m,'rays':rays,'heading_policy':'same_four_as_original',
        'jitter_discount_radius_m':model.resolution,'current_boundary_cells':len(current),
        'current_boundary_world_xy':[grid.world(r,c) for r,c in sorted(current)],
        'all_candidates':selected,'rejected':audit,'candidate_limit':limit,
        'frontier_clusters':[{k:v for k,v in c.items() if k!='points'} for c in identities['clusters']],
        'visibility_is_safety_certificate':False,'actual_map_gain_m2':None}


def compare(catalog,output):
    meta=json.loads(catalog.read_text());map_path=catalog.with_suffix('.map.npz')
    with np.load(map_path) as z:
        data=z['data'].copy();resolution=float(z['resolution']);origin=tuple(z['origin'])
    pose=meta['robot_pose'];offset=meta['base_to_lidar_tf'];scan_range=meta['range_m']
    model=AggressiveMap(data,resolution,origin,sensor_offset=offset[:2],sensor_range_m=scan_range)
    original=replace(model.settings);variants={}
    def save():
        result={'scope':'paired_readonly_on_identical_archived_map_pose_sensor_tf; no motion or live writes',
            'source_catalog':str(catalog),'source_map':str(map_path),
            'source_map_sha256':hashlib.sha256(map_path.read_bytes()).hexdigest(),
            'robot_pose':pose,'base_to_lidar_tf':offset,'range_m':scan_range,
            'unchanged_safety_settings':asdict(original),'variants':variants,'navigation_commands_sent':0}
        output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    for radius in (3.,4.5,6.,8.):
        start=time.monotonic();model.settings=replace(original,maximum_frontier_distance=radius)
        audit=[];candidates,info=model.candidates(pose,[],regional=True,audit=audit)
        variants['radius_'+str(radius)]={'candidates':candidates,'generation':info,'audit':audit,'wall_s':time.monotonic()-start,
                                       'change':'maximum_frontier_distance_only','path_validation':'NOT_YET_PERFORMED'}
        save();print(json.dumps({'variant':'radius_'+str(radius),'count':len(candidates),'generation':info,'wall_s':time.monotonic()-start}),flush=True)
    model.settings=original;start=time.monotonic()
    candidates,diagnostics=visible_search(model,pose,offset,scan_range)
    variants['visible_boundary']={'candidates':candidates,'diagnostics':diagnostics,'wall_s':time.monotonic()-start,
                                 'path_validation':'NOT_YET_PERFORMED'}
    save();print(json.dumps({'variant':'visible_boundary','count':len(candidates),'filters':diagnostics['filter_counts'],'wall_s':time.monotonic()-start}),flush=True)
    return output


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--catalog',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();compare(a.catalog,a.output)
