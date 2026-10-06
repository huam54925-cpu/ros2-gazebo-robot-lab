#!/usr/bin/env python3
"""Read-only fixed-scene evaluation; never imported by navigation or MCP.

Reports nominal registered free-space coverage for indoor_mapping.sdf. Unknown
and occupied map cells do not count as observed free space. The fixed interior
free-space denominator excludes static boxes at lidar height. This is not a
collision or passage-success evaluator, and SLAM registration error remains.
"""
import argparse
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np


def reference(world, resolution=.05):
    root=ET.parse(world).getroot()
    # This evaluator deliberately refuses other scenes until their observable
    # region has been specified, rather than guessing from expanding map bounds.
    models={m.get('name'):m for m in root.findall('./world/model')}
    required={'room_perimeter','corridor_north','corridor_south','vehicle'}
    if not required<=models.keys():raise ValueError('unsupported_coverage_scene')
    x,y=np.meshgrid(np.arange(-12+resolution/2,12,resolution),np.arange(-10+resolution/2,10,resolution))
    free=np.ones(x.shape,dtype=bool)
    def pose(el):return np.array([float(v) for v in el.findtext('pose','0 0 0 0 0 0').split()])
    for name,model in models.items():
        if name in ('vehicle','ground_plane'):continue
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                box=collision.find('geometry/box/size')
                if box is None:raise ValueError('unsupported_coverage_geometry')
                p=pose(model)+pose(link)+pose(collision)
                if np.any(abs(p[3:])>1e-8):raise ValueError('rotated_coverage_geometry')
                size=np.array([float(v) for v in box.text.split()])
                if p[2]-size[2]/2<=.9<=p[2]+size[2]/2:
                    free &= ~((abs(x-p[0])<=size[0]/2)&(abs(y-p[1])<=size[1]/2))
    vehicle=models['vehicle'];spawn=pose(vehicle)
    wheel=[pose(vehicle.find("link[@name='%s']"%n)) for n in ('left_wheel','right_wheel')]
    axle=spawn[:2]+(wheel[0][:2]+wheel[1][:2])/2
    if np.any(abs(spawn[3:])>1e-8):raise ValueError('rotated_initial_pose')
    return np.column_stack((x[free],y[free]))-axle, int(free.sum())*resolution**2


def measure(grid, points):
    o=grid['origin'];c,s=math.cos(o[2]),math.sin(o[2]);delta=points-np.array(o[:2])
    ix=np.floor((c*delta[:,0]+s*delta[:,1])/grid['resolution']).astype(int)
    iy=np.floor((-s*delta[:,0]+c*delta[:,1])/grid['resolution']).astype(int)
    h,w=grid['data'].shape;inside=(ix>=0)&(iy>=0)&(ix<w)&(iy<h)
    values=np.full(len(points),-1,dtype=int);values[inside]=grid['data'][iy[inside],ix[inside]]
    return {'observed_free_fraction':float(np.mean((values>=0)&(values<25))),
            'known_fraction':float(np.mean(values>=0)),
            'occupied_on_truth_free_fraction':float(np.mean(values>=65))}


def main():
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile,DurabilityPolicy
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--world',required=True);p.add_argument('--output',required=True)
    args=p.parse_args();points,area=reference(args.world)
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('offline_coverage_observer');last=[0.]
    def callback(msg):
        if time.monotonic()-last[0]<2:return
        last[0]=time.monotonic();o=msg.info.origin;q=o.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        grid={'origin':[o.position.x,o.position.y,yaw],'resolution':msg.info.resolution,
              'data':np.asarray(msg.data).reshape(msg.info.height,msg.info.width)}
        row={'wall_unix_s':time.time(),'map_stamp_sim_s':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
             'reference_free_area_m2':area,'registration':'nominal_initial_axle_map_origin',
             'scope':'fixed_scene_free_space; not collision or passage validation',**measure(grid,points)}
        with out.open('a') as f:f.write(json.dumps(row)+'\n')
        np.savez_compressed(out.with_suffix('.map.npz'),data=grid['data'],origin=grid['origin'],resolution=grid['resolution'])
    sub=node.create_subscription(OccupancyGrid,'/map',callback,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
