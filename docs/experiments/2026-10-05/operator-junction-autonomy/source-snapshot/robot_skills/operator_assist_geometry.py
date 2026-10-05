"""Pose sweeps for local operator translation/rotation, including reverse travel.

Unlike Nav2 prediction, these commands have a prescribed body heading. Reverse
translation must not invent a 180 degree turn. No motion or permission here.
"""
import math
import numpy as np
from observation import body_check, navigation_footprint
from path_safety import PathSafety


def sweep(grid, start, distance=0., turn=0., clearance=1.48):
    if (not all(math.isfinite(v) for v in [*start,distance,turn,clearance])
            or abs(distance)>6 or abs(turn)>math.pi or (distance and turn)
            or clearance not in (1.48,2.15)):
        raise ValueError('invalid_operator_segment')
    body,padding=navigation_footprint()
    radius=max(float(np.linalg.norm(body,axis=1).max())+padding,.7057095)
    count=max(1,math.ceil((abs(distance)+radius*abs(turn))/.05))
    model=PathSafety(grid['data'],grid['resolution'],grid['origin'],grid.get('map_version','snapshot'))
    minimum=math.inf
    for f in np.linspace(0.,1.,count+1):
        pose=[start[0]+f*distance*math.cos(start[2]),start[1]+f*distance*math.sin(start[2]),start[2]+f*turn]
        c=model.evaluate([pose],pose,pose,(-.7057095,0.),clearance)
        value=c['predicted_min_clearance_m']
        if value is not None:minimum=min(minimum,value)
        if not c['safe']:return {'safe':False,'reason':'operator_path_clearance','witness':pose,'clearance':c}
        b=body_check(grid['data'],grid['resolution'],grid['origin'],pose,body,padding)
        if not b['safe']:return {'safe':False,'reason':'operator_body_nonfree','witness':pose,'body':b}
    return {'safe':True,'minimum_clearance_m':minimum if math.isfinite(minimum) else None,
            'samples':count+1,'sample_travel_bound_m':.05,'required_clearance_m':clearance,
            'distance_m':distance,'turn_rad':turn,'end_pose':pose}
