#!/usr/bin/env python3
"""Compare saved occupied cells with this world's static box surfaces.

Allows one rigid registration for arbitrary SLAM map origin. This measures
observed obstacle alignment, not free-space accuracy or complete coverage.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def main():
    ap=argparse.ArgumentParser();ap.add_argument('map_yaml');ap.add_argument('--world',required=True)
    args=ap.parse_args();path=Path(args.map_yaml);meta=yaml.safe_load(path.read_text())
    pixels=np.asarray(Image.open(path.parent/meta['image']))
    h,w=pixels.shape;r=meta['resolution'];ox,oy,oa=meta['origin']
    rows,cols=np.nonzero(pixels<80)
    xy=np.column_stack(((cols+.5)*r,(h-rows-.5)*r))
    rot=np.array([[math.cos(oa),-math.sin(oa)],[math.sin(oa),math.cos(oa)]])
    xy=xy@rot.T+[ox,oy]
    boxes=[]
    for model in ET.parse(args.world).getroot().findall('./world/model'):
        if model.get('name') in ['vehicle','ground_plane']:continue
        def pos(el):return [float(v) for v in el.findtext('pose','0 0 0 0 0 0').split()]
        mp=pos(model)
        for link in model.findall('link'):
            lp=pos(link)
            for collision in link.findall('collision'):
                box=collision.find('geometry/box/size')
                if box is None:continue
                cp=pos(collision);size=[float(v) for v in box.text.split()]
                assert abs(mp[5])+abs(lp[5])+abs(cp[5])<1e-8, 'Only axis-aligned boxes supported'
                x,y=mp[0]+lp[0]+cp[0],mp[1]+lp[1]+cp[1]
                boxes.append((x-size[0]/2,x+size[0]/2,y-size[1]/2,y+size[1]/2))
    def transform(p,params):
        dx,dy,a=params;c,s=math.cos(a),math.sin(a)
        return p@np.array([[c,s],[-s,c]])+np.array([dx,dy])
    def distances(p):
        best=np.full(len(p),np.inf);x,y=p.T
        for a,b,c,d in boxes:
            outside=np.hypot(np.maximum(np.maximum(a-x,x-b),0),np.maximum(np.maximum(c-y,y-d),0))
            inside=np.minimum.reduce([abs(x-a),abs(x-b),abs(y-c),abs(y-d)])
            dist=np.where((x>=a)&(x<=b)&(y>=c)&(y<=d),inside,outside)
            best=np.minimum(best,dist)
        return best
    # Nominal map origin is initial axle, then fit small rigid frame differences.
    params=np.array([-7.4457175,-5.,0.]);sample=xy[::max(1,len(xy)//2500)]
    def loss(p):return float(np.mean(np.minimum(distances(transform(sample,p)),.4)**2))
    initial=distances(transform(xy,params));best=loss(params)
    for step in [.1,.05,.02,.01,.005]:
        for _ in range(30):
            improved=False
            for axis,delta in enumerate([step,step,step/10]):
                for sign in [-1,1]:
                    trial=params.copy();trial[axis]+=sign*delta
                    if abs(trial[0]+7.4457175)>1 or abs(trial[1]+5)>1 or abs(trial[2])>math.radians(5):continue
                    score=loss(trial)
                    if score<best:params,best,improved=trial,score,True
            if not improved:break
    registered=transform(xy,params);errors=distances(registered)
    report={'occupied_cells':len(xy),'known_cells':int(np.sum(pixels!=205)),
        'nominal_surface_error_p95_m':float(np.quantile(initial,.95)),
        'rigid_map_to_world':{'x':params[0],'y':params[1],'yaw_deg':math.degrees(params[2])},
        'registered_surface_error_median_m':float(np.median(errors)),
        'registered_surface_error_p95_m':float(np.quantile(errors,.95)),
        'occupied_cells_within_15cm_fraction':float(np.mean(errors<=.15)),
        'scope':'Observed occupied-cell alignment after one rigid registration; not full coverage or free-space validation.'}
    path.with_suffix('.quality.json').write_text(json.dumps(report,indent=2)+'\n')
    fig,ax=plt.subplots(figsize=(10,9))
    for a,b,c,d in boxes:ax.add_patch(Rectangle((a,c),b-a,d-c,facecolor='#dbe3ed',edgecolor='#63748c'))
    sc=ax.scatter(*registered.T,c=np.minimum(errors,.3),s=3,cmap='plasma',vmin=0,vmax=.3)
    fig.colorbar(sc,ax=ax,label='Distance to static surface (m)')
    ax.set(xlim=(-13,13),ylim=(-11,11),xlabel='Gazebo world X (m)',ylabel='Gazebo world Y (m)',
        title='Observed SLAM obstacles vs. scene geometry\nOne rigid map registration; unobserved areas are not validated')
    ax.set_aspect('equal');ax.grid(alpha=.2);fig.tight_layout();fig.savefig(path.with_suffix('.quality.png'),dpi=140)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
