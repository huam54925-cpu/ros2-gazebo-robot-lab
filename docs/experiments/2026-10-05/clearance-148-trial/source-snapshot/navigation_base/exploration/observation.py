"""Pure map geometry for finite observations; no robot commands or model calls."""
import ast
import math
from pathlib import Path
import numpy as np
import yaml
from path_safety import PathSafety, SAMPLE_STEP_M, angle_delta


def map_semantics(data,resolution):
    """Observed map classes only; unknown extent outside the array is unbounded."""
    data=np.asarray(data);area=resolution**2
    counts={'free':int(((data>=0)&(data<25)).sum()),'occupied':int((data>=65).sum()),
            'unknown':int((data<0).sum()),'uncertain_occupancy':int(((data>=25)&(data<65)).sum())}
    return {'cells':counts,'area_m2':{k:n*area for k,n in counts.items()},
            'unknown_remaining_in_grid':counts['unknown']>0,
            'outside_grid':'unobserved_extent_not_counted',
            'task_scope':'not_configured','historical_unknown_causes':'not_inferable_from_occupancy_alone',
            'navigation_rule':'only_known_free_body_and_path',
            'exploration_rule':'seek_safe_free_views_of_unknown', 'world_coverage':None}


def navigation_footprint(path=None):
    path=path or Path(__file__).resolve().parents[1]/'navigation/nav2.yaml'
    config=yaml.safe_load(Path(path).read_text())
    specs=[config[name][name]['ros__parameters'] for name in ('local_costmap','global_costmap')]
    polygons=[ast.literal_eval(c['footprint']) for c in specs]
    if polygons[0]!=polygons[1] or specs[0]['footprint_padding']!=specs[1]['footprint_padding']:
        raise ValueError('inconsistent_navigation_footprint')
    return np.asarray(polygons[0],dtype=float),float(specs[0]['footprint_padding'])


def body_sweep(data,resolution,origin,path,start,goal):
    """Check padded body along both planned poses and navigation turn prediction."""
    body,padding=navigation_footprint();radius=float(np.linalg.norm(body,axis=1).max())+padding
    checked=0
    for chain in (navigation_prediction(path,start,goal),[start,*path,goal]):
        for a,b in zip(chain,chain[1:]):
            turn=angle_delta(a[2],b[2]);travel=math.dist(a[:2],b[:2])+radius*abs(turn)
            count=max(1,math.ceil(travel/SAMPLE_STEP_M))
            for f in np.linspace(0,1,count+1):
                pose=[a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1]),a[2]+f*turn];checked+=1
                evidence=body_check(data,resolution,origin,pose,body,padding)
                if not evidence['safe']:
                    return {'safe':False,'reason':'body_sweep_nonfree','witness_pose':pose,
                            'checked_samples':checked,'cell_evidence':evidence}
    return {'safe':True,'reason':'clear','checked_samples':checked,'sample_travel_bound_m':SAMPLE_STEP_M}


def rotation_path(pose, signed_angle, radius):
    if not all(math.isfinite(v) for v in [*pose,signed_angle,radius]) or radius<0:
        raise ValueError('invalid_rotation')
    count=max(1,math.ceil(abs(signed_angle)*max(radius,.01)/SAMPLE_STEP_M))
    return [[pose[0],pose[1],pose[2]+signed_angle*i/count] for i in range(count+1)]


def polygon_square_distances(polygon, centers, resolution):
    """Exact Euclidean distance from a convex polygon to closed grid squares.

    Separating axes detect overlap. Otherwise the minimum lies on a vertex/edge
    pair. Grid half-diagonals are not added: the square itself is tested.
    """
    half=resolution/2; low=centers-half; high=centers+half
    overlap=np.ones(len(centers),dtype=bool)
    edges=np.roll(polygon,-1,axis=0)-polygon
    axes=[np.array([1.,0.]),np.array([0.,1.]),*np.column_stack((-edges[:,1],edges[:,0]))]
    for axis in axes:
        projection=polygon@axis; middle=centers@axis; extent=half*np.abs(axis).sum()
        overlap &= (middle+extent>=projection.min()) & (middle-extent<=projection.max())
    delta=np.maximum(np.maximum(low[:,None,:]-polygon[None,:,:],
                                polygon[None,:,:]-high[:,None,:]),0)
    distance2=(delta*delta).sum(axis=2).min(axis=1)
    corners=centers[:,None,:]+half*np.array([[-1,-1],[1,-1],[1,1],[-1,1]])
    for a,edge in zip(polygon,edges):
        t=np.clip(((corners-a)*edge).sum(axis=2)/np.dot(edge,edge),0,1)
        delta=corners-(a+t[:,:,None]*edge)
        distance2=np.minimum(distance2,(delta*delta).sum(axis=2).min(axis=1))
    return np.where(overlap,0.,np.sqrt(distance2))


def body_check(data,resolution,origin,pose,footprint,padding):
    """Padded body versus non-free cell squares, including inter-sample travel."""
    ox,oy,a=origin; c,s=math.cos(a),math.sin(a)
    center=np.array([c*(pose[0]-ox)+s*(pose[1]-oy),-s*(pose[0]-ox)+c*(pose[1]-oy)])
    a=pose[2]-a; c,s=math.cos(a),math.sin(a)
    polygon=footprint@np.array([[c,s],[-s,c]])+center
    # Sampling bounds body travel by SAMPLE_STEP_M; a nearest sample is at most
    # half a step away. Retain the configured padding in addition to that bound.
    margin=padding+SAMPLE_STEP_M/2
    h,w=data.shape; lower=polygon.min(axis=0)-margin; upper=polygon.max(axis=0)+margin
    info={'method':'exact_convex_polygon_to_cell_square_v1','padding_m':padding,
          'motion_sample_margin_m':SAMPLE_STEP_M/2,'required_body_margin_m':margin}
    if np.any(lower<=0) or np.any(upper>=np.array([w,h])*resolution):
        return {**info,'safe':False,'reason':'body_outside_known_map','blocking_cells':[]}
    low=np.maximum(0,np.floor(lower/resolution).astype(int))
    high=np.minimum([w-1,h-1],np.floor(upper/resolution).astype(int))
    subset=data[low[1]:high[1]+1,low[0]:high[0]+1]
    iy,ix=np.nonzero((subset<0)|(subset>=25)); ix=ix+low[0];iy=iy+low[1]
    if not len(ix):return {**info,'safe':True,'reason':'clear','blocking_cells':[]}
    centers=np.column_stack((ix+.5,iy+.5))*resolution
    distances=polygon_square_distances(polygon,centers,resolution)
    blocked=np.flatnonzero(distances<=margin+1e-9)
    cells=[{'cell_xy':[int(ix[i]),int(iy[i])],'value':int(data[iy[i],ix[i]]),
            'distance_to_unpadded_body_m':float(distances[i])} for i in blocked[:12]]
    values=data[iy[blocked],ix[blocked]]
    return {**info,'safe':not len(blocked),'reason':'body_sweep_nonfree' if len(blocked) else 'clear',
            'blocking_cell_count':len(blocked),'blocking_cells':cells,
            'blocking_counts':{'unknown':int((values<0).sum()),'occupied':int((values>=65).sum()),
                               'uncertain':int(((values>=25)&(values<65)).sum())}}


def body_safe(data,resolution,origin,pose,footprint,padding):
    return body_check(data,resolution,origin,pose,footprint,padding)['safe']


def check_rotation(data,resolution,origin,version,pose,angle,offset,footprint,padding):
    radius=max(float(np.linalg.norm(footprint,axis=1).max())+padding,math.hypot(*offset))
    path=rotation_path(pose,angle,radius)
    clearance=PathSafety(data,resolution,origin,version).evaluate(path,pose,path[-1],offset)
    body=all(body_safe(data,resolution,origin,p,footprint,padding) for p in path)
    return {**clearance,'safe':clearance['safe'] and body,'body_sweep_checked':True,
            'body_sweep_safe':body,'reason':clearance['reason'] if not clearance['safe'] else ('clear' if body else 'body_sweep_nonfree'),
            'signed_angle_rad':angle,'rotation_samples':len(path)}


class Visibility:
    """World-fixed rays; zero-offset 360 degree lidar gains nothing from yaw alone."""
    def __init__(self,data,resolution,origin,offset,sensor_range_m=None):
        self.origin,self.offset=origin,offset
        self.sensor_range_m=(float(sensor_range_m) if sensor_range_m is not None and
                             math.isfinite(sensor_range_m) and sensor_range_m>0 else None)
        # Bounded optimistic information proxy, not a full scan simulator.
        self.proxy_range_m=min(8.,self.sensor_range_m) if self.sensor_range_m else 8.
        self.cache={}
        scale=max(1,round(.2/resolution)); self.resolution=scale*resolution
        h,w=data.shape; padded=np.full((math.ceil(h/scale)*scale,math.ceil(w/scale)*scale),-1)
        padded[:h,:w]=data
        blocks=padded.reshape(padded.shape[0]//scale,scale,padded.shape[1]//scale,scale).transpose(0,2,1,3)
        self.occupied=np.any(blocks>=65,axis=(2,3))
        self.blocked=np.any(blocks>=25,axis=(2,3)); self.unknown=np.any(blocks<0,axis=(2,3))
    def sensor_xy(self,pose):
        a=pose[2]; sx,sy=self.offset
        x=pose[0]+math.cos(a)*sx-math.sin(a)*sy; y=pose[1]+math.sin(a)*sx+math.cos(a)*sy
        ox,oy,oa=self.origin; c,s=math.cos(oa),math.sin(oa)
        return c*(x-ox)+s*(y-oy),-s*(x-ox)+c*(y-oy)
    def trace(self,pose):
        x,y=self.sensor_xy(pose);oa=self.origin[2]
        key=(x,y)
        if key in self.cache: return self.cache[key]
        seen=set(); occluded=set(); uncertain=set(); h,w=self.unknown.shape
        edge_rays=0
        for world_angle in np.linspace(0,2*math.pi,90,endpoint=False):
            a=world_angle-oa; c,s=math.cos(a),math.sin(a); depth=0.; previous=None
            wall=False; ambiguous=False
            for d in np.arange(0,self.proxy_range_m,self.resolution/2):
                ix,iy=int(math.floor((x+d*c)/self.resolution)),int(math.floor((y+d*s)/self.resolution))
                if not(0<=ix<w and 0<=iy<h):
                    edge_rays+=1
                    break
                if (iy,ix)==previous: continue
                previous=(iy,ix)
                wall |= bool(self.occupied[iy,ix])
                ambiguous |= bool(self.blocked[iy,ix])
                # A mixed coarse block with a known obstacle is never credited
                # as visible unknown. Continue rays only to explain occlusion.
                if self.blocked[iy,ix]: continue
                if self.unknown[iy,ix]:
                    if wall: occluded.add((iy,ix))
                    elif ambiguous or depth>=2: uncertain.add((iy,ix))
                    else:
                        seen.add((iy,ix)); depth+=self.resolution
        self.cache[key]={'visible':frozenset(seen),'occluded':frozenset(occluded-seen),
                         'uncertain':frozenset(uncertain-seen-occluded), 'map_edge_rays':edge_rays}
        return self.cache[key]
    def visible(self,pose):
        return self.trace(pose)['visible']
    def analyze(self,pose,current=None):
        evidence=self.trace(pose);area=self.resolution**2
        visible,occluded,uncertain=(evidence[k] for k in ('visible','occluded','uncertain'))
        out_of_range=None
        if current is not None and self.sensor_range_m:
            x,y=self.sensor_xy(current)
            out_of_range=sum(math.hypot((ix+.5)*self.resolution-x,(iy+.5)*self.resolution-y)
                             > self.sensor_range_m for iy,ix in visible)
        kind=('UNKNOWN_OUT_OF_RANGE' if visible and out_of_range is not None and out_of_range>len(visible)/2 else
              'UNKNOWN_OPEN' if visible else 'UNKNOWN_OCCLUDED' if occluded else 'UNKNOWN_UNCERTAIN')
        total=len(visible|occluded|uncertain)
        return {'unknown_type':kind, 'classification_basis':'sampled_map_rays_from_this_sensor_pose',
                'historical_unknown_cause':'UNDETERMINED',
                'visible_unknown_area_proxy_m2':len(visible)*area,
                'occluded_unknown_area_proxy_m2':len(occluded)*area,
                'uncertain_unknown_area_proxy_m2':len(uncertain)*area,
                'occluded_fraction_proxy':len(occluded)/total if total else None,
                'beyond_current_sensor_range_proxy_m2':out_of_range*area if out_of_range is not None else None,
                'sensor_range_m':self.sensor_range_m,'proxy_range_m':self.proxy_range_m,
                'unknown_depth_cap_m':2.,'map_edge_rays':evidence['map_edge_rays'],
                'gain_confidence':'optimistic_proxy_not_observed_area',
                'unknown_is_traversable':False}
    def novel(self,poses,current,recent=()):
        baseline=self.visible(current)
        for p in recent[-8:]: baseline |= self.visible(p)
        visible=set()
        for p in poses: visible |= self.visible(p)
        return len(visible-baseline)*self.resolution**2


def navigation_prediction(path,start,goal):
    """Forward segment headings plus starting/final turns, matching clearance assumptions."""
    nodes=[list(start)]
    for point in [*path,goal]:
        previous=nodes[-1]
        if math.dist(previous[:2],point[:2])>1e-6:
            bearing=math.atan2(point[1]-previous[1],point[0]-previous[0])
            nodes.extend([[*previous[:2],bearing],[*point[:2],bearing]])
    nodes.append(list(goal))
    return nodes


def estimated_time(length,poses):
    turn=sum(abs(angle_delta(a[2],b[2])) for a,b in zip(poses,poses[1:]))
    return length/.15+turn/.20+4.


def action_time_estimate(length, poses, resolution):
    """Timing-only correction of a reversed sub-cell start connector.

    Clearance still uses every original pose. Do not simplify real corners or
    assume translation and turning overlap. Rates remain conservative; the two
    2026-10-04 windows are evidence for this artifact, not a fitted speed model.
    """
    raw_turn=sum(abs(angle_delta(a[2],b[2])) for a,b in zip(poses,poses[1:]))
    timed=list(poses)
    segments=[i for i in range(1,len(poses)) if math.dist(poses[i-1][:2],poses[i][:2])>1e-6]
    removed=0.
    if len(segments)>=2:
        first,second=segments[:2]
        connector=math.dist(poses[first-1][:2],poses[first][:2])
        reversal=abs(angle_delta(poses[first][2],poses[second][2]))
        if connector<=resolution*math.sqrt(2)+1e-6 and reversal>math.pi/2:
            timed=[poses[0],*poses[first+1:]]
            removed=connector
    turn=sum(abs(angle_delta(a[2],b[2])) for a,b in zip(timed,timed[1:]))
    components={'translation_sim_s':length/.15,'turning_sim_s':turn/.20,
                'planning_validation_sim_s':2.,'stop_verification_sim_s':1.,'feedback_wait_sim_s':4.}
    expected=sum(components.values())
    reserve=max(3.,.10*(components['translation_sim_s']+components['turning_sim_s']))
    return {'method':'start_connector_corrected_v1','components':components,
            'raw_turn_rad':raw_turn,'timing_turn_rad':turn,'ignored_start_connector_m':removed,
            'legacy_sim_s':estimated_time(length,poses),'expected_sim_s':expected,
            'budget_reserve_sim_s':reserve,'budget_sim_s':expected+reserve,
            'scope':'worker_before_to_after; catalog_and_model_measured_separately',
            'calibration':'provisional_two_recorded_windows; not_a_deadline_guarantee',
            'safety_path_modified':False}


def map_change(before,after, allow_fractional=False):
    """Register aligned grids, including integer translations / boundary expansion."""
    if before['frame']!=after['frame'] or abs(before['resolution']-after['resolution'])>1e-8 or abs(angle_delta(before['origin'][2],after['origin'][2]))>1e-6:
        return {'gain_status':'UNCOMPARABLE','reason':'map_geometry_changed'}
    r=before['resolution']; dx=after['origin'][0]-before['origin'][0]; dy=after['origin'][1]-before['origin'][1]
    a=before['origin'][2]; shift=np.array([math.cos(a)*dx+math.sin(a)*dy,-math.sin(a)*dx+math.cos(a)*dy])/r
    aligned = bool(np.allclose(shift,np.round(shift),atol=1e-4,rtol=0))
    if not aligned and not allow_fractional: return {'gain_status':'UNCOMPARABLE','reason':'unaligned_map_origin'}
    if aligned: shift=np.round(shift)
    base=np.floor(shift).astype(int); fraction=shift-base
    old,new=before['data']>=0,after['data']>=0
    free=(after['data']>=0)&(after['data']<25)
    occupied=after['data']>=65
    ambiguous=(after['data']>=25)&(after['data']<65)
    def overlap(mask):
        shared=0.
        # Exact rectangle intersection areas for equally resolved, parallel
        # piecewise-constant occupancy grids. No nearest-neighbor rounding.
        for ix,wx in ((0,1-fraction[0]),(1,fraction[0])):
            for iy,wy in ((0,1-fraction[1]),(1,fraction[1])):
                if wx*wy==0: continue
                sx,sy=base+[ix,iy]
                x0,y0=max(0,sx),max(0,sy); x1,y1=min(old.shape[1],sx+new.shape[1]),min(old.shape[0],sy+new.shape[0])
                if x1>x0 and y1>y0:
                    shared+=float(np.sum(old[y0:y1,x0:x1]&mask[y0-sy:y1-sy,x0-sx:x1-sx]))*wx*wy
        return shared
    shared=overlap(new)
    gained=max(0.,float(new.sum())-shared)*r*r; lost=max(0.,float(old.sum())-shared)*r*r
    return {'gain_status':'FEEDBACK_INCOMPLETE','observed_new_known_area_m2':gained,
            'observed_lost_known_area_m2':lost,'net_known_area_change_m2':gained-lost,
            'observed_new_free_area_m2':max(0.,float(free.sum())-overlap(free))*r*r,
            'observed_new_occupied_area_m2':max(0.,float(occupied.sum())-overlap(occupied))*r*r,
            'observed_new_ambiguous_area_m2':max(0.,float(ambiguous.sum())-overlap(ambiguous))*r*r,
            'measurement_status':'REGISTERED_MAP_CHANGE',
            'registration':'integer_grid' if aligned else 'fractional_cell_area_overlap',
            'grid_shift_cells':shift.tolist(),
            'accepted_scan_evidence':'UNAVAILABLE',
            'note':'Map-window change, not causal gain. Fractional origins/SLAM corrections can create boundary changes; no LOW_GAIN claim.'}
