"""Low-speed simulation guard using laser returns and the swept body polygon.

This has no map/world-truth input. Both commanded and measured motion are checked
through a reaction + braking horizon. The guard is not a navigation controller.
"""
import math
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent/'exploration'))
from observation import navigation_footprint, polygon_square_distances
from safety_contract import scan_state
from robot_contract import CONTRACT


def scan_points(scan):
    _,valid=scan_state(scan)
    if (not valid or len(scan.ranges)!=360 or
            abs(scan.angle_increment*(len(scan.ranges)-1)-2*math.pi)>.05):
        raise ValueError('unexpected_scan_geometry')
    ranges=np.asarray(scan.ranges)
    if np.any(np.isnan(ranges)) or np.any(ranges==-np.inf) or np.any(ranges<scan.range_min):
        raise ValueError('invalid_scan_beam')
    angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
    hit=np.isfinite(ranges)&(ranges<=scan.range_max)
    sx,sy,sa=CONTRACT['scan_in_base']
    points=np.column_stack((ranges[hit]*np.cos(angles[hit]+sa)+sx,
                            ranges[hit]*np.sin(angles[hit]+sa)+sy))
    return points


def predicted_pose(v,w,t):
    if abs(w)<1e-9:return np.array([v*t,0.,0.])
    return np.array([v*math.sin(w*t)/w,v*(1-math.cos(w*t))/w,w*t])


def check_sweep(points,command,measured=(0.,0.),scan_age=0.):
    from simulation_override import active,unchecked
    if active():return unchecked()
    if not all(math.isfinite(v) for v in [*command,*measured,scan_age]) or scan_age<0:
        return {'safe':False,'reason':'invalid_motion_feedback'}
    body,_=navigation_footprint()
    radius=float(np.linalg.norm(body,axis=1).max())
    points=np.asarray(points,dtype=float).reshape(-1,2)
    if not np.all(np.isfinite(points)):return {'safe':False,'reason':'invalid_scan_points'}
    minimum=math.inf;witness=None;count=0
    # .075 body allowance + .005 continuous motion bound. At the nearest walls
    # angular beam spacing is < 2 cm; a 1 cm half-square covers that sampling.
    margin=(CONTRACT['body_padding_m']+CONTRACT['motion_sample_step_m']/2
            +CONTRACT['guard_sample_step_m']/2)
    for v,w in (command,measured):
        horizon=CONTRACT['reaction_time_s']+scan_age+max(
            abs(v)/CONTRACT['linear_deceleration_m_s2'],
            abs(w)/CONTRACT['angular_deceleration_rad_s2'])
        travel=(abs(v)+radius*abs(w))*horizon
        steps=max(1,math.ceil(travel/CONTRACT['guard_sample_step_m']))
        for t in np.linspace(0,horizon,steps+1):
            pose=predicted_pose(v,w,t);c,s=math.cos(pose[2]),math.sin(pose[2])
            polygon=body@np.array([[c,s],[-s,c]])+pose[:2]
            # Only nearby returns can intersect this body. Keep all angular sectors.
            relevant=points[np.linalg.norm(points-pose[:2],axis=1)<=radius+.2]
            if not len(relevant):continue
            gaps=polygon_square_distances(polygon,relevant,CONTRACT['laser_hit_square_m'])
            i=int(np.argmin(gaps));gap=float(gaps[i]);count+=1
            if gap<minimum:minimum=gap;witness={'predicted_pose':pose.tolist(),'hit_base_xy':relevant[i].tolist(),'time_s':float(t)}
            if gap<=margin:
                return {'safe':False,'reason':'scan_body_sweep','body_clearance_m':gap,
                        'required_margin_m':margin,'witness':witness,'samples':count}
    return {'safe':True,'reason':'clear','body_clearance_m':minimum if math.isfinite(minimum) else None,
            'required_margin_m':margin,'samples':count,'witness':witness}
