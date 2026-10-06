"""Small optional lidar/body sweep gate, independent of Nav2 and old explorers.

Not a certified collision predictor: discrete lidar samples and constant-twist
projection have blind spots. No global map, candidate veto or recovery chain.
"""
import math
import numpy as np

# Measured chassis polygon relative to the differential-drive axle, metres.
BODY = np.array([[-1.73,-.53],[-.32,-.96],[.32,-.96],[.32,.96],[-.32,.96],[-1.73,.53]])
SCAN_IN_BASE = (-.7057095, 0.)


class LidarSweep:
    def __init__(self, margin=.02, horizon=.5):
        self.margin=margin;self.horizon=horizon

    def check(self, points, linear, angular):
        points=np.asarray(points,dtype=float).reshape(-1,2)
        if not np.all(np.isfinite(points)):
            return {'allowed':False,'reason':'invalid_lidar_points'}
        for t in np.linspace(0,self.horizon,21):
            a=angular*t;c=math.cos(a);s=math.sin(a)
            xy=np.array([linear*t,0.]) if abs(angular)<1e-9 else np.array([linear*s/angular,linear*(1-c)/angular])
            local=(points-xy)@np.array([[c,-s],[s,c]])
            inside=np.ones(len(points),dtype=bool)
            for p,q in zip(BODY,np.roll(BODY,-1,axis=0)):
                edge=q-p;d=local-p
                inside &= edge[0]*d[:,1]-edge[1]*d[:,0] >= -self.margin*np.linalg.norm(edge)
            if inside.any():
                return {'allowed':False,'reason':'lidar_body_sweep','prediction_s':float(t),
                        'point_base_xy':points[np.flatnonzero(inside)[0]].tolist()}
        return {'allowed':True,'reason':'clear_lidar_sweep'}

    def evaluate(self, scan, odom_pose, odom_at_scan, now_sim_s, linear, angular):
        stamp=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
        if not -.05 <= now_sim_s-stamp <= .5 or odom_at_scan is None:
            return {'allowed':False,'reason':'scan_feedback_unavailable_or_stale'}
        ranges=np.asarray(scan.ranges);angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
        if np.any(np.isnan(ranges)) or np.any(ranges<scan.range_min):
            return {'allowed':False,'reason':'invalid_lidar_ranges'}
        hits=np.isfinite(ranges)&(ranges<=scan.range_max)
        points=np.column_stack((ranges[hits]*np.cos(angles[hits])+SCAN_IN_BASE[0],ranges[hits]*np.sin(angles[hits])+SCAN_IN_BASE[1]))
        def rotation(a):return np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
        # Compensate movement since scan reception using the paired odometry.
        points=(points@rotation(odom_at_scan[2]).T+np.array(odom_at_scan[:2])-np.array(odom_pose[:2]))@rotation(odom_pose[2])
        return self.check(points,linear,angular)
