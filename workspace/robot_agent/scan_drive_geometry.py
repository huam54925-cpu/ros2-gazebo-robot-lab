"""Measurement interpretation for the direct scan / half-range experiment."""
import math


def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))


def scan_rays(ranges, angle_min, increment, range_min, range_max, yaw_map):
    rows=[]
    for index,distance in enumerate(ranges):
        if math.isnan(distance) or distance<range_min:continue
        no_return=not math.isfinite(distance) or distance>range_max
        distance=range_max if no_return else float(distance)
        rows.append({'beam_index':index,'heading_map_rad':wrap(yaw_map+angle_min+index*increment),
                     'visible_range_m':distance,'half_range_m':distance/2,'no_return_within_range':no_return})
    return rows


def direction_summary(rays):
    bins={}
    for ray in rays:
        index=int((wrap(ray['heading_map_rad'])+math.pi)/(2*math.pi)*24)%24
        if index not in bins or ray['visible_range_m']>bins[index]['visible_range_m']:bins[index]=ray
    return [dict(bins[k],sector=k) for k in sorted(bins)]


def half_range_goal(rays, heading, pose):
    if not rays or not math.isfinite(heading):raise ValueError('valid_scan_and_heading_required')
    ray=min(rays,key=lambda r:abs(wrap(r['heading_map_rad']-heading)))
    heading=ray['heading_map_rad'];distance=ray['half_range_m']
    return {**ray,'distance_m':distance,'requested_heading_map_rad':heading,
            'target_map_xy':[pose[0]+distance*math.cos(heading),pose[1]+distance*math.sin(heading)],
            'rule':'half_of_measured_range_in_ai_selected_direction; not a collision clearance'}
