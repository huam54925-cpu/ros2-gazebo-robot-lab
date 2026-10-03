"""Information-seeking observation goals; all inputs are the current SLAM map."""
import math
import numpy as np
from frontier import FrontierMap, Settings, components


def settings():
    # Replace an all-heading unknown-space circle with the actual oriented body.
    # Known-obstacle clearance and the independent velocity guard are unchanged.
    return Settings(unknown_clearance=0.35, visited_radius=1.0, minimum_goal_distance=1.2,
                    maximum_frontier_distance=3.0, maximum_candidates=12)


def utility(candidate, length, robot_yaw):
    bearing = math.atan2(candidate['dy'], candidate['dx'])
    turn = abs(math.atan2(math.sin(bearing-robot_yaw), math.cos(bearing-robot_yaw)))
    return candidate['estimated_gain_m2'] / (1.0 + 0.12*length + 0.30*turn)


class AggressiveMap(FrontierMap):
    def __init__(self, data, resolution, origin, config=None):
        super().__init__(data, resolution, origin, config or settings())
        self.raw = np.asarray(data)
        self.raw_resolution = resolution

    def is_safe(self, x, y, heading=0.0):
        if not super().is_safe(x, y):
            return False
        # Padded convex footprint from the navigation configuration. Check every
        # fine-cell center in its envelope, with an extra grid diagonal margin.
        footprint = np.array([[-1.73,-.53],[-.32,-.96],[.32,-.96],
                              [.32,.96],[-.32,.96],[-1.73,.53]])
        ox, oy, a = self.origin
        c, s = math.cos(a), math.sin(a)
        center = np.array([c*(x-ox)+s*(y-oy), -s*(x-ox)+c*(y-oy)])
        a = heading-a
        c, s = math.cos(a), math.sin(a)
        polygon = footprint @ np.array([[c,s],[-s,c]]) + center
        margin = .05 + self.raw_resolution*math.sqrt(2)
        low = np.floor((polygon.min(axis=0)-margin)/self.raw_resolution).astype(int)
        high = np.ceil((polygon.max(axis=0)+margin)/self.raw_resolution).astype(int)
        h, w = self.raw.shape
        if low.min()<0 or high[0]>=w or high[1]>=h:
            return False
        xx, yy = np.meshgrid(np.arange(low[0],high[0]+1),np.arange(low[1],high[1]+1))
        points = np.column_stack(((xx.ravel()+.5)*self.raw_resolution,(yy.ravel()+.5)*self.raw_resolution))
        inside = np.ones(len(points), dtype=bool)
        for p, q in zip(polygon, np.roll(polygon,-1,axis=0)):
            edge = q-p
            cross = edge[0]*(points[:,1]-p[1])-edge[1]*(points[:,0]-p[0])
            inside &= cross >= -margin*np.linalg.norm(edge)
        cells = self.raw[yy.ravel()[inside],xx.ravel()[inside]]
        return bool(len(cells) and np.all((cells>=0)&(cells<25)))

    def gain(self, x, y):
        # Optimistic visibility proxy, NOT calibrated entropy/information gain.
        # Known walls terminate rays. Only 2 m of unknown depth is credited,
        # since visibility beyond unknown obstacles cannot be assumed.
        seen = set()
        h, w = self.unknown.shape
        for angle in np.linspace(0,2*math.pi,90,endpoint=False):
            unknown_depth = 0.0
            for distance in np.arange(self.resolution,8.0,self.resolution):
                iy, ix = self.cell(x+distance*math.cos(angle),y+distance*math.sin(angle))
                if not (0<=iy<h and 0<=ix<w) or self.occupied[iy,ix]:
                    break
                if self.unknown[iy,ix]:
                    seen.add((iy,ix))
                    unknown_depth += self.resolution
                    if unknown_depth>=2.0:
                        break
        return len(seen)*self.resolution**2

    def candidates(self, robot, excluded=()):
        groups = [g for g in components(self.frontier)
                  if len(g)*self.resolution>=self.settings.minimum_frontier_length]
        info = {'frontier_clusters':len(groups),'safe_cells':int(self.safe.sum()),
                'frontier_cells':int(self.frontier.sum())}
        if not groups:
            return [], info
        boundary = self.world(np.concatenate(groups))
        points = self.world(np.argwhere(self.safe))
        candidates = []
        sampled = []
        for point in points:
            delta = point-np.asarray(robot[:2]); distance = float(np.linalg.norm(delta))
            if distance<self.settings.minimum_goal_distance:
                continue
            if any(np.linalg.norm(point-np.asarray(p[:2]))<self.settings.visited_radius for p in excluded):
                continue
            if any(np.linalg.norm(point-p)<.8 for p in sampled):
                continue
            distances = np.linalg.norm(boundary-point,axis=1)
            target = boundary[np.argmin(distances)]
            if distances.min()>self.settings.maximum_frontier_distance:
                continue
            heading = math.atan2(target[1]-point[1],target[0]-point[0])
            if not self.is_safe(*point,heading):
                continue
            sampled.append(point)
            candidate = {'x':float(point[0]),'y':float(point[1]),'yaw':heading,
                         'dx':float(delta[0]),'dy':float(delta[1]),
                         'euclidean_distance_m':distance,'frontier_distance_m':float(distances.min()),
                         'estimated_gain_m2':self.gain(*point)}
            candidate['preplan_score'] = utility(candidate,distance,robot[2])
            candidates.append(candidate)
        candidates.sort(key=lambda g:g['preplan_score'],reverse=True)
        selected = candidates[:self.settings.maximum_candidates]
        for i, candidate in enumerate(selected):
            candidate['id'] = f'information-{i+1}'
        info['oriented_candidates_before_cap'] = len(candidates)
        return selected, info
