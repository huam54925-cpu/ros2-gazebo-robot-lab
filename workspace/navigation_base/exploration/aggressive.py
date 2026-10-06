"""Information-seeking observation goals; all inputs are the current SLAM map."""
import math
import time
import numpy as np
from frontier import FrontierMap, Settings, components


def settings():
    # Replace an all-heading unknown-space circle with the actual oriented body.
    # Known-obstacle clearance and the independent velocity guard are unchanged.
    from safety_profile import FOOTPRINT_MODE
    return Settings(obstacle_clearance=0. if FOOTPRINT_MODE else 2.65,
                    unknown_clearance=0. if FOOTPRINT_MODE else 0.35, visited_radius=1.0, minimum_goal_distance=1.2,
                    maximum_frontier_distance=3.0, maximum_candidates=12)


def utility(candidate, length, robot_yaw):
    bearing = math.atan2(candidate['dy'], candidate['dx'])
    turn = abs(math.atan2(math.sin(bearing-robot_yaw), math.cos(bearing-robot_yaw)))
    return candidate['estimated_gain_m2'] / (1.0 + 0.12*length + 0.30*turn)


class AggressiveMap(FrontierMap):
    def __init__(self, data, resolution, origin, config=None, sensor_offset=(0.,0.), sensor_range_m=None):
        super().__init__(data, resolution, origin, config or settings())
        self.raw = np.asarray(data)
        self.raw_resolution = resolution
        from observation import Visibility,navigation_footprint
        self.visibility = Visibility(data,resolution,origin,sensor_offset,sensor_range_m)
        self.navigation_body,self.navigation_padding=navigation_footprint()

    def is_safe(self, x, y, heading=0.0):
        if not super().is_safe(x, y):
            return False
        from observation import body_safe
        return body_safe(self.raw,self.raw_resolution,self.origin,[x,y,heading],
                         self.navigation_body,self.navigation_padding)

    def gain(self, x, y, heading=0.):
        return len(self.visibility.visible([x,y,heading]))*self.visibility.resolution**2

    def candidates(self, robot, excluded=(), regional=False, candidate_offset=0, audit=None, progress=None):
        groups = [g for g in components(self.frontier)
                  if len(g)*self.resolution>=self.settings.minimum_frontier_length]
        info = {'frontier_clusters':len(groups),'safe_cells':int(self.safe.sum()),
                'frontier_cells':int(self.frontier.sum()),
                'safe_cell_filters':{'known_free_cells':int(self.free.sum()),
                    'obstacle_clearance_rejected':int((self.free&(self.obstacle_distance<self.settings.obstacle_clearance)).sum()),
                    'known_body_margin_rejected':int((self.free&(self.known_clearance<self.settings.unknown_clearance)).sum()),
                    'counts_may_overlap':True},
                'pose_filter_counts':dict(too_near=0,previously_attempted=0,spatial_sample_duplicate=0,
                                         frontier_too_far=0,no_safe_heading=0,selected_positions=0)}
        if not groups:
            return [], info
        boundary = self.world(np.concatenate(groups))
        points = self.world(np.argwhere(self.safe))
        candidates = []
        sampled = []
        def record(point,reason,**extra):
            if audit is not None:audit.append({'position':[float(v) for v in point],'stage':'pose_generation','reason':reason,**extra})
        feedback_at=time.monotonic()
        for point in points:
            if progress is not None and time.monotonic()-feedback_at>=.25:
                progress();feedback_at=time.monotonic()
            delta = point-np.asarray(robot[:2]); distance = float(np.linalg.norm(delta))
            if distance<self.settings.minimum_goal_distance:
                info['pose_filter_counts']['too_near']+=1
                record(point,'MINIMUM_GOAL_DISTANCE')
                continue
            if any(np.linalg.norm(point-np.asarray(p[:2]))<self.settings.visited_radius for p in excluded):
                info['pose_filter_counts']['previously_attempted']+=1
                record(point,'LEGACY_VISITED_EXCLUSION')
                continue
            if any(np.linalg.norm(point-p)<.8 for p in sampled):
                info['pose_filter_counts']['spatial_sample_duplicate']+=1
                record(point,'SPATIAL_SAMPLING_DUPLICATE')
                continue
            distances = np.linalg.norm(boundary-point,axis=1)
            target = boundary[np.argmin(distances)]
            if distances.min()>self.settings.maximum_frontier_distance:
                info['pose_filter_counts']['frontier_too_far']+=1
                record(point,'MAXIMUM_FRONTIER_DISTANCE')
                continue
            bearing = math.atan2(target[1]-point[1],target[0]-point[0])
            # A frontier is a viewing direction, not a goal inside unknown.
            # Test alternate orientations of the asymmetric body/lidar at the
            # same known-free position before discarding that observation site.
            headings = [bearing,bearing+math.pi/2,bearing-math.pi/2,robot[2]]
            headings = [a for a in headings if self.is_safe(*point,a)]
            if not headings:
                info['pose_filter_counts']['no_safe_heading']+=1
                record(point,'NO_SAFE_FOOTPRINT_HEADING')
                continue
            heading = max(headings,key=lambda a:(self.visibility.novel([[*point,a]],robot),
                                                 self.gain(*point,a)))
            sampled.append(point)
            candidate = {'x':float(point[0]),'y':float(point[1]),'yaw':heading,
                         'dx':float(delta[0]),'dy':float(delta[1]),
                         'euclidean_distance_m':distance,'frontier_distance_m':float(distances.min()),
                         'estimated_gain_m2':self.gain(*point,heading),
                         'unknown_analysis':self.visibility.analyze([*point,heading],robot),
                         'gain_model':'sensor_offset_occlusion_aware_rays_v2'}
            candidate['preplan_score'] = utility(candidate,distance,robot[2])
            candidates.append(candidate)
            record(point,'POSE_PROPOSAL',candidate=dict(candidate))
            info['pose_filter_counts']['selected_positions']+=1
        candidates.sort(key=lambda g:g['preplan_score'],reverse=True)
        if regional:
            from regions import diverse_candidates
            ordered = diverse_candidates(candidates, len(candidates))
            selected = ordered[candidate_offset:candidate_offset+self.settings.maximum_candidates]
        else:
            selected = candidates[:self.settings.maximum_candidates]
        for i, candidate in enumerate(selected):
            candidate['id'] = f'information-{i+1}'
        info['oriented_candidates_before_cap'] = len(candidates)
        info['shortlist_truncated'] = len(candidates) > len(selected)
        info['regional_diversity_enabled'] = regional
        return selected, info
