"""Laser motion compensation using stamped odometry; no extrapolated free space."""
import math
from collections import deque
import numpy as np
from robot_contract import CONTRACT


def angle(a):return math.atan2(math.sin(a),math.cos(a))


class OdomHistory:
    def __init__(self):
        self.rows=deque(maxlen=5000)

    def add(self, stamp, pose, frame):
        if frame!=CONTRACT['odom_frame'] or not all(math.isfinite(v) for v in [stamp,*pose]):
            self.rows.clear();raise ValueError('scan_pose_invalid_odometry')
        if self.rows and stamp<self.rows[-1][0]:
            self.rows.clear();raise ValueError('scan_pose_clock_reset')
        if self.rows and stamp==self.rows[-1][0]:self.rows.pop()
        self.rows.append((stamp,tuple(pose)))
        while len(self.rows)>2 and self.rows[1][0]<stamp-5.:self.rows.popleft()

    def at(self, stamp):
        if not self.rows or stamp<self.rows[0][0] or stamp>self.rows[-1][0]:
            raise ValueError('scan_pose_history_unavailable')
        for t,p in reversed(self.rows):
            if t==stamp:return p
        for (a,p),(b,q) in zip(self.rows,list(self.rows)[1:]):
            if a<=stamp<=b:
                if b-a>CONTRACT['scan_pose_gap_s']:raise ValueError('scan_pose_history_gap')
                f=(stamp-a)/(b-a)
                return (p[0]+f*(q[0]-p[0]),p[1]+f*(q[1]-p[1]),p[2]+f*angle(q[2]-p[2]))
        raise ValueError('scan_pose_history_unavailable')


def compensated_points(scan, history, now_sim):
    from footprint_guard import scan_points
    points=scan_points(scan)
    stamp=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
    increment=float(scan.time_increment)
    if not math.isfinite(increment) or increment<0:raise ValueError('scan_pose_invalid_increment')
    if not history.rows:raise ValueError('scan_pose_history_unavailable')
    current_stamp,current=history.rows[-1]
    limit=CONTRACT['profiles']['footprint_075']['scan_timeout_s']
    if not 0<=now_sim-stamp<=limit or not 0<=now_sim-current_stamp<=limit:
        raise ValueError('scan_pose_feedback_stale')
    # Reject a missing acquisition pose even if every beam is infinity.
    history.at(stamp)
    history.at(stamp+increment*(len(scan.ranges)-1))
    ranges=np.asarray(scan.ranges)
    indices=np.flatnonzero(np.isfinite(ranges)&(ranges<=scan.range_max))
    c,s=math.cos(current[2]),math.sin(current[2]);output=[];poses={}
    for point,index in zip(points,indices):
        beam_stamp=stamp+int(index)*increment
        if beam_stamp not in poses:poses[beam_stamp]=history.at(beam_stamp)
        p=poses[beam_stamp]
        ca,sa=math.cos(p[2]),math.sin(p[2])
        wx=p[0]+ca*point[0]-sa*point[1];wy=p[1]+sa*point[0]+ca*point[1]
        dx,dy=wx-current[0],wy-current[1]
        output.append([c*dx+s*dy,-s*dx+c*dy])
    return np.asarray(output,dtype=float).reshape(-1,2),{
        'scan_stamp_sim_s':stamp,'compensated_to_odom_sim_s':current_stamp,
        'remaining_pose_age_sim_s':now_sim-current_stamp,'method':'per_beam_odometry_interpolation'}
