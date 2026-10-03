"""Map-only frontier extraction and conservative observation-goal generation.

No ROS, saved map, simulator geometry or ground-truth input. Planning remains
Nav2's responsibility. Coarse cells are free only if every fine cell is free.
"""
from dataclasses import dataclass, asdict
import heapq
import math
import numpy as np


@dataclass
class Settings:
    coarse_resolution: float = 0.20
    minimum_frontier_length: float = 0.8
    minimum_unknown_area: float = 2.0
    obstacle_clearance: float = 2.65  # 1.9 m lidar guard + 0.706 m axle offset + margin
    unknown_clearance: float = 1.9  # conservative rotating body envelope
    maximum_frontier_distance: float = 4.0
    minimum_goal_distance: float = 1.0
    visited_radius: float = 1.5
    maximum_candidates: int = 12


def distance_field(mask, resolution):
    """8-neighbor chamfer distance, conservative Euclidean lower bound.

An octile path is up to 8.24% longer than Euclidean. Scaling by cos(pi/8)
    and subtracting a cell diagonal makes clearance filtering conservative.
    """
    h,w=mask.shape
    d=np.full((h,w),np.inf);queue=[]
    for y,x in np.argwhere(mask):d[y,x]=0.;queue.append((0.,int(y),int(x)))
    heapq.heapify(queue)
    steps=[(dy,dx,math.hypot(dy,dx)*resolution) for dy in [-1,0,1] for dx in [-1,0,1] if dy or dx]
    while queue:
        v,y,x=heapq.heappop(queue)
        if v>d[y,x]:continue
        for dy,dx,cost in steps:
            yy,xx=y+dy,x+dx
            if 0<=yy<h and 0<=xx<w and v+cost<d[yy,xx]:
                d[yy,xx]=v+cost;heapq.heappush(queue,(v+cost,yy,xx))
    return np.maximum(0.,d*math.cos(math.pi/8)-resolution*math.sqrt(2))


def components(mask):
    seen=np.zeros_like(mask,dtype=bool);out=[];h,w=mask.shape
    for y,x in np.argwhere(mask):
        if seen[y,x]:continue
        todo=[(int(y),int(x))];seen[y,x]=True;points=[]
        while todo:
            yy,xx=todo.pop();points.append((yy,xx))
            for dy in [-1,0,1]:
                for dx in [-1,0,1]:
                    y2,x2=yy+dy,xx+dx
                    if 0<=y2<h and 0<=x2<w and mask[y2,x2] and not seen[y2,x2]:
                        seen[y2,x2]=True;todo.append((y2,x2))
        out.append(np.asarray(points))
    return out


class FrontierMap:
    def __init__(self, data, resolution, origin, settings=None):
        self.settings=settings or Settings();self.origin=origin
        self.scale=max(1,round(self.settings.coarse_resolution/resolution))
        self.resolution=self.scale*resolution
        data=np.asarray(data);h,w=data.shape;s=self.scale
        # Partial blocks are unknown, never silently assumed free.
        padded=np.full((math.ceil(h/s)*s,math.ceil(w/s)*s),-1,dtype=np.int16);padded[:h,:w]=data
        blocks=padded.reshape(padded.shape[0]//s,s,padded.shape[1]//s,s).transpose(0,2,1,3)
        self.occupied=np.any(blocks>=65,axis=(2,3))
        self.free=np.all((blocks>=0)&(blocks<25),axis=(2,3))
        self.unknown=~(self.free|self.occupied)
        # Boundary outside the map is unknown. Padding prevents wraparound edges.
        significant_unknown=np.zeros_like(self.unknown)
        for group in components(self.unknown):
            if len(group)*self.resolution**2>=self.settings.minimum_unknown_area:
                significant_unknown[group[:,0],group[:,1]]=True
        u=np.pad(significant_unknown,1,constant_values=True)
        neighbor=u[:-2,1:-1]|u[2:,1:-1]|u[1:-1,:-2]|u[1:-1,2:]
        self.frontier=self.free&neighbor
        self.obstacle_distance=distance_field(self.occupied,self.resolution)
        nonfree=np.pad(~self.free,1,constant_values=True)
        self.known_clearance=distance_field(nonfree,self.resolution)[1:-1,1:-1]
        self.safe=self.free&(self.obstacle_distance>=self.settings.obstacle_clearance)&(self.known_clearance>=self.settings.unknown_clearance)

    def world(self, cells):
        cells=np.asarray(cells);local=np.column_stack(((cells[:,1]+.5)*self.resolution,(cells[:,0]+.5)*self.resolution))
        x,y,a=self.origin;c,s=math.cos(a),math.sin(a)
        return local@np.array([[c,s],[-s,c]])+[x,y]

    def cell(self,x,y):
        ox,oy,a=self.origin;c,s=math.cos(a),math.sin(a);dx,dy=x-ox,y-oy
        return int(math.floor((-s*dx+c*dy)/self.resolution)),int(math.floor((c*dx+s*dy)/self.resolution))

    def is_safe(self,x,y):
        iy,ix=self.cell(x,y)
        return 0<=iy<self.safe.shape[0] and 0<=ix<self.safe.shape[1] and bool(self.safe[iy,ix])

    def candidates(self,robot,excluded=()):
        config=self.settings
        groups=[g for g in components(self.frontier) if len(g)*self.resolution>=config.minimum_frontier_length]
        cells=np.argwhere(self.safe);xy=self.world(cells)
        if len(xy)==0:return [],{'frontier_clusters':len(groups),'safe_cells':0}
        distances=np.linalg.norm(xy-np.asarray(robot[:2]),axis=1)
        valid=distances>=config.minimum_goal_distance
        for point in excluded:valid &= np.linalg.norm(xy-np.asarray(point[:2]),axis=1)>=config.visited_radius
        xy=xy[valid];distances=distances[valid]
        if not len(xy):return [],{'frontier_clusters':len(groups),'safe_cells':int(self.safe.sum())}
        candidates=[]
        for group in groups:
            boundary=self.world(group)
            # Minimum distance to a real frontier cell, not a centroid inside a wall.
            nearest=np.full(len(xy),np.inf);nearest_index=np.zeros(len(xy),dtype=int)
            for i,p in enumerate(boundary):
                d=np.linalg.norm(xy-p,axis=1);better=d<nearest;nearest[better]=d[better];nearest_index[better]=i
            eligible=np.flatnonzero(nearest<=config.maximum_frontier_distance)
            if not len(eligible):continue
            # Prefer observation positions near the boundary; keep several alternatives.
            rank=eligible[np.argsort(nearest[eligible]+.15*distances[eligible])]
            chosen=[]
            for index in rank:
                point=xy[index]
                if any(np.linalg.norm(point-other)<1.5 for other in chosen):continue
                chosen.append(point);target=boundary[nearest_index[index]]
                candidates.append({'x':float(point[0]),'y':float(point[1]),
                    'yaw':math.atan2(target[1]-point[1],target[0]-point[0]),
                    'frontier_center':boundary.mean(axis=0).tolist(),'frontier_cells':len(group),
                    'frontier_distance_m':float(nearest[index]),'euclidean_distance_m':float(distances[index])})
                if len(chosen)==3:break
        candidates.sort(key=lambda g:g['euclidean_distance_m'])
        unique=[]
        for candidate in candidates:
            if any(math.hypot(candidate['x']-p['x'],candidate['y']-p['y'])<.75 for p in unique):continue
            candidate['id']=f"frontier-{len(unique)+1}";unique.append(candidate)
            if len(unique)>=config.maximum_candidates:break
        return unique,{'frontier_clusters':len(groups),'safe_cells':int(self.safe.sum()),'frontier_cells':int(self.frontier.sum())}
