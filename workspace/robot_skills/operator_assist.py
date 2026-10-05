"""Explicit local simulation operator assistance; never exposed to MCP/model.

Prescribed low speed reverse/rotate/forward segments go through the existing
scan velocity guard. Nav2 is paused, ownership locked, and every segment and
short braking horizon are checked against the current sensor-built map.
"""
import argparse,json,math,signal,sys,time
from pathlib import Path
from types import SimpleNamespace as NS
W=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(W),str(W/'navigation_base/exploration'),str(W/'navigation_base')]
from explore import Explorer,rclpy,yaw,ManageLifecycleNodes,BASE_FRAME,SCAN_FRAME
from robot_skills.store import Store,ACTIVE
from robot_skills.operator_assist_geometry import sweep
from geometry_msgs.msg import Twist
from rclpy.signals import SignalHandlerOptions


def wrap(a):return math.atan2(math.sin(a),math.cos(a))


class Assist(Explorer):
    def __init__(self, output, target, reverse_m=1.):
        super().__init__(NS(output=str(output),policy='aggressive',wall_budget=360,max_goals=1,goal_timeout=100))
        self.store=Store();self.target=target;self.reverse_m=reverse_m;self.pub=self.node.create_publisher(Twist,'/model/vehicle/cmd_vel',1)
        self.report.update(selector='local_operator_assistance',target=target,segments=[],path_clearance_m=1.48,
                           guard_threshold_m=1.9,autonomous_exploration=False)
        self.owner=str(output);self.claimed=False
    def odom_cb(self,msg):
        super().odom_cb(msg)
        p=msg.pose.pose.position;self.odom_pose=[p.x,p.y,yaw(msg.pose.pose.orientation)]
    def grid(self):
        m=self.latest['map'];o=m.info.origin
        return dict(data=self.latest['data'],resolution=m.info.resolution,origin=[o.position.x,o.position.y,yaw(o.orientation)],map_version=self.latest['map_version'])
    def health(self):
        try:super().health()
        except RuntimeError as error:
            if str(error) not in ('localization_stale','localization_unavailable'):raise
            for _ in range(128):rclpy.spin_once(self.node,timeout_sec=0.)
            super().health()
    def zero(self):self.pub.publish(Twist())
    def safe_health(self):
        self.health()
        if self.store.meta('stop_latched',False) or self.output.with_suffix('.cancel').exists():raise RuntimeError('operator_stop')
        if self.store.meta('active_mission'):raise RuntimeError('mission_conflict')
        if self.distance>8 or time.monotonic()-self.started>360:raise RuntimeError('operator_budget')
        if self.latest['nearest']<1.9:raise RuntimeError('guard_stop')
    def halt(self):
        until=time.monotonic()+.6
        while time.monotonic()<until:self.zero();self.spin_once()
        self.zero()
        return self.stop_confirmed()
    def segment(self,distance=0.,turn=0.):
        self.zero();self.safe_health()
        check=sweep(self.grid(),self.pose(),distance,turn,1.48)
        row={'start':self.sample(),'requested_distance_m':distance,'requested_turn_rad':turn,'preflight':check}
        self.report['segments'].append(row);self.checkpoint()
        if not check['safe']:raise RuntimeError(check['reason'])
        anchor=self.odom_pose[:];previous=anchor[2];rotated=0.;began=time.monotonic();checkpoint_at=0.
        while True:
            self.spin_once()
            for _ in range(32):rclpy.spin_once(self.node,timeout_sec=0.)
            self.safe_health()
            now=time.monotonic();p=self.odom_pose
            rotated+=wrap(p[2]-previous);previous=p[2]
            travelled=(p[0]-anchor[0])*math.cos(anchor[2])+(p[1]-anchor[1])*math.sin(anchor[2])
            cross=abs(-(p[0]-anchor[0])*math.sin(anchor[2])+(p[1]-anchor[1])*math.cos(anchor[2]))
            remaining=turn-rotated if turn else distance-travelled
            tolerance=.02
            if abs(remaining)<=tolerance:break
            if remaining*(turn if turn else distance)<0:raise RuntimeError('segment_overshoot')
            if now-began>120:raise RuntimeError('segment_timeout')
            if distance and (cross>.12 or abs(rotated)>.10):raise RuntimeError('operator_tracking_error')
            # A check covers the next 0.2 m / 0.15 rad. Guard command expiry
            # is 0.5 s and commanded speeds are <=0.12 m/s and 0.18 rad/s.
            probe=math.copysign(min(abs(remaining),.15 if turn else .2),remaining)
            version=self.latest['map_version']
            horizon=sweep(self.grid(),self.pose(),0. if turn else probe,probe if turn else 0.,1.48)
            self.safe_health()
            if not horizon['safe']:
                self.zero();row['invalidation']=horizon;raise RuntimeError(horizon['reason'])
            if version!=self.latest['map_version']:
                self.zero();continue
            cmd=Twist()
            speed=min(.18 if turn else .12,max(.035,abs(remaining)*.7))
            if turn:cmd.angular.z=math.copysign(speed,remaining)
            else:cmd.linear.x=math.copysign(speed,remaining)
            self.pub.publish(cmd)
            if now-checkpoint_at>1:
                row.update(travelled_m=travelled,rotated_rad=rotated,last_pose=self.pose())
                self.traces.append(self.sample());self.checkpoint();checkpoint_at=now
        if not self.halt():raise RuntimeError('stop_unconfirmed')
        row.update(status='succeeded',after=self.sample());self.checkpoint()
    def run(self):
        try:
            with self.store.transaction() as db:
                if self.store._meta(db,'active_mission') or self.store._meta(db,'active_operator_assistance') or self.store._meta(db,'stop_latched',False):raise RuntimeError('not_available_for_operator_assist')
                tasks=[json.loads(r[0]) for r in db.execute('SELECT body FROM tasks')]
                if any(t['status'] in ACTIVE or (t['status'] in ('indeterminate','stop_unconfirmed') and not t.get('resolved')) for t in tasks):raise RuntimeError('unresolved_task')
                self.store._set_meta(db,'active_operator_assistance',self.owner);self.claimed=True
            if not self.manager.wait_for_service(timeout_sec=3):raise RuntimeError('nav2_manager_unavailable')
            req=ManageLifecycleNodes.Request();req.command=req.PAUSE
            if not self.wait(self.manager.call_async(req),15).success:raise RuntimeError('navigation_pause_failed')
            until=time.monotonic()+15
            while not all(k in self.latest for k in ('map','odom_at','scan_at')) or not self.buffer.can_transform('map',BASE_FRAME,rclpy.time.Time()):
                if time.monotonic()>until:raise RuntimeError('feedback_unavailable')
                self.spin_once()
            self.observe(1);self.safe_health()
            t=self.buffer.lookup_transform(BASE_FRAME,SCAN_FRAME,rclpy.time.Time())
            if math.dist([t.transform.translation.x,t.transform.translation.y],[-.7057095,0])>.001:raise RuntimeError('sensor_offset_changed')
            if not self.halt():raise RuntimeError('not_stationary')
            self.report['before']=self.sample();self.save_map('before')
            if not 3<math.dist(self.pose()[:2],self.target)<6:raise RuntimeError('unexpected_operator_start')
            self.segment(distance=-self.reverse_m)
            for _ in range(8):
                pose=self.pose();distance=math.dist(pose[:2],self.target)
                if distance<.22:
                    self.report['status']='succeeded';break
                turn=wrap(math.atan2(self.target[1]-pose[1],self.target[0]-pose[0])-pose[2])
                if abs(turn)>.03:self.segment(turn=turn)
                self.segment(distance=min(1.,max(0.,distance-.10)))
            else:raise RuntimeError('operator_segment_limit')
        except (Exception,KeyboardInterrupt) as error:
            self.report.update(status='aborted',reason=str(error) or 'interrupted')
        finally:
            stopped=False
            if self.claimed:
                self.zero();stopped=self.halt()
            self.report['stopped']=stopped
            if self.claimed:
                self.store.set_meta('stop_latched',True)
                if stopped:self.store.set_meta('active_operator_assistance',None)
            try:
                self.report['after']=self.sample();self.save_map('after')
                self.report['target_error_m']=math.dist(self.pose()[:2],self.target)
            except Exception:pass
            self.report['verified_autonomous_gain_m2']=None;self.checkpoint()
            if self.claimed:self.store.set_meta('last_operator_assistance',{'selector':'local_operator','status':self.report['status'],'stopped':stopped,'target':self.target,'after':self.report.get('after'),'distance_odom_m':self.distance,'not_autonomous_exploration':True})
            self.event_file.close();self.node.destroy_node();self.lock.close();self.motion_lock.close()
        return self.report


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--target',nargs=2,type=float,required=True);p.add_argument('--reverse-m',type=float,default=1.);a=p.parse_args()
    if not math.isfinite(a.reverse_m) or not 0<a.reverse_m<=1.2:p.error('reverse distance must be in (0,1.2]')
    if not all(math.isfinite(v) for v in a.target):p.error('finite coordinates required')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM,signal.default_int_handler);signal.signal(signal.SIGINT,signal.default_int_handler)
    try:result=Assist(a.output,a.target,a.reverse_m).run();print(json.dumps({'status':result['status'],'stopped':result['stopped'],'target_error_m':result.get('target_error_m')}))
    finally:rclpy.shutdown()

if __name__=='__main__':main()
