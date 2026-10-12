"""Container-owned Gazebo lifetime; JSON lines RPC, no learning dependencies."""
import collections
from decimal import Decimal
import gc
import json
import math
import os
import selectors
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from gz.transport import Node
from gz.msgs.laserscan_pb2 import LaserScan
from gz.msgs.odometry_pb2 import Odometry
from gz.msgs.pose_v_pb2 import Pose_V
from gz.msgs.world_stats_pb2 import WorldStatistics


def ns(t): return t.sec*1_000_000_000+t.nsec

def rpy(q):
    return (math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)),
            math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x)))),
            math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

class ServiceUnavailable(RuntimeError):
    """No transport reply: only read-only queries may be retried."""

class Worker:
    def __init__(self, req):
        self.episode=req['episode_id']; self.action=-1; self.current_ns=0
        self.node=Node(); self.lock=threading.Lock(); self.stats=None
        self.cache={k:collections.OrderedDict() for k in ('lidar','odom','truth')}
        self.proc=None; self.log=None; self.topics=[]; self.closed=False; self.client=None
        self.tmp=tempfile.TemporaryDirectory(prefix='local-drive-')
        self.physics_ns=1_000_000
        try:
            self.client=subprocess.Popen([sys.executable,str(Path(__file__).with_name('service_client.py'))],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
            for typ,topic,cb in ((LaserScan,'/scan',self.scan),(Odometry,'/model/vehicle/odometry',self.odom),
                                  (Pose_V,'/world/demo/dynamic_pose/info',self.truth),
                                  (WorldStatistics,'/world/demo/stats',self.statistics)):
                if not self.node.subscribe(typ,topic,cb): raise RuntimeError('subscribe '+topic)
                self.topics.append(topic)
            tree=ET.parse('/src/workspace/navigation_base/worlds/indoor_mapping.sdf')
            world=tree.getroot().find('world')
            for old in world.findall('physics'): world.remove(old)
            physics=ET.SubElement(world,'physics',name='sync_1ms',type='ignored')
            ET.SubElement(physics,'max_step_size').text='0.001'
            ET.SubElement(physics,'real_time_factor').text='1.0'
            model=next(m for m in world.findall('model') if m.get('name')=='vehicle')
            for plugin in model.findall('plugin'):
                if 'diff-drive' in plugin.get('filename',''): model.remove(plugin)
            plugin=ET.SubElement(model,'plugin',filename='/sync/libLocalDriveSync.so',name='local_drive::SyncDrive')
            ET.SubElement(plugin,'episode_id').text=str(self.episode)
            path=Path(self.tmp.name)/'world.sdf';tree.write(path)
            self.log=open(Path(self.tmp.name)/'gazebo.log','w+')
            self.proc=subprocess.Popen(['gz','sim','-s','--headless-rendering','--seed',str(req['seed']),'-v','2',str(path)],stdout=self.log,stderr=subprocess.STDOUT)
            self.wait(lambda:self.stats is not None,40,'initial stats')
            if self.stats['ns']!=0 or not self.stats['paused']: raise RuntimeError('world not initially paused at zero')
            # Zero action 0 is committed before a 600 ms settling interval.
            ack=self.apply(0.,0.,0)
            for target in (200_000_000,400_000_000,600_000_000):
                self.advance_physics(200,target)
                self.frame(target,{}) # force an aligned sample at each settling boundary
            receipt=self.receipt(ack,600,600_000_000)
            self.current_ns=600_000_000
            self.initial=self.frame(self.current_ns,receipt)
            self.require_stopped(self.initial,receipt)
            render_log=Path("/root/.gz/rendering/ogre2.log")
            self.initial["evaluation"]["renderer"]=[line.strip() for line in render_log.read_text(errors="replace").splitlines() if "GL_RENDERER" in line] if render_log.exists() else []
            with self.lock: settled_before=self.cache["truth"].get(400_000_000)
            if settled_before is None or not self.stable_pose(settled_before["pose"],self.initial["evaluation"]["truth_pose"]):
                raise RuntimeError("reset truth drift: "+str(settled_before)+" -> "+str(self.initial["evaluation"]["truth_pose"]))
        except BaseException:
            self.close();raise

    def put(self,key,t,value):
        with self.lock:
            self.cache[key][t]=value
            while len(self.cache[key])>5000: self.cache[key].popitem(last=False)
    def scan(self,m): self.put('lidar',ns(m.header.stamp),{'ranges':list(m.ranges),'range_min':m.range_min,'range_max':m.range_max})
    def odom(self,m):
        ep=next((d.value[0] for d in m.header.data if d.key=='episode_id'),None)
        if ep!=str(self.episode): return
        self.put('odom',ns(m.header.stamp),{'pose':[m.pose.position.x,m.pose.position.y,rpy(m.pose.orientation)[2]],'velocity':[m.twist.linear.x,m.twist.angular.z]})
    def truth(self,m):
        p=next((p for p in m.pose if p.name=='vehicle'),None)
        if p:
            roll,pitch,yaw=rpy(p.orientation)
            self.put('truth',ns(m.header.stamp),{'pose':[p.position.x+0.5542825*math.cos(yaw),p.position.y+0.5542825*math.sin(yaw),yaw],'roll':roll,'pitch':pitch})
    def statistics(self,m):
        with self.lock:self.stats={'ns':ns(m.sim_time),'paused':m.paused,'iterations':m.iterations}
    def wait(self,predicate,timeout,reason):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if self.proc and self.proc.poll() is not None: raise RuntimeError('Gazebo exited: '+self.log_text())
            result=predicate()
            if result:return result
            time.sleep(.01)
        raise TimeoutError(reason)
    def log_text(self):
        if not self.log:return ''
        self.log.flush();self.log.seek(0);return self.log.read()[-3000:]
    def service(self,name,reqtype,reptype,request):
        self.client.stdin.write(json.dumps(dict(name=name,reqtype=reqtype,reptype=reptype,request=request))+'\n');self.client.stdin.flush()
        with selectors.DefaultSelector() as selector:
            selector.register(self.client.stdout,selectors.EVENT_READ)
            if not selector.select(5):raise RuntimeError('service client unresponsive; refusing further requests')
        line=self.client.stdout.readline()
        if not line:raise RuntimeError('service client exited')
        response=json.loads(line)
        if not response['ok']:raise ServiceUnavailable('native response unavailable: '+name+' '+response.get('error',''))
        if reptype=='StringMsg' and not response['data']:raise ServiceUnavailable('empty StringMsg: '+name)
        return response['data']
    def status(self):
        # Read-only retry is safe; never retry command or world multi_step.
        for attempt in range(3):
            try:return json.loads(self.service('/local_drive/status','gz.msgs.Empty','StringMsg',''))
            except ServiceUnavailable as exc:
                print(f'status query attempt {attempt+1}: {exc}',file=sys.stderr,flush=True)
                if attempt==2:raise
                time.sleep(.05)
    def apply(self,v,w,action,fault=None):
        command=f'{self.episode} {action} {v:.17g} {w:.17g}'
        if not self.service('/local_drive/command','gz.msgs.StringMsg','Boolean','data: '+json.dumps(command)):
            raise RuntimeError('actuator command rejected')
        def applied():
            ack=self.status()
            if fault=='missing_ack':return False
            if ack['action_id']!=action:return False
            if not ack['healthy'] or not ack['committed'] or not ack['paused'] or ack['episode_id']!=self.episode or ack['applied_ns']!=self.current_ns:
                raise RuntimeError('invalid applied ACK: '+str(ack))
            if abs(ack['v']-v)>1e-12 or abs(ack['w']-w)>1e-12: raise RuntimeError('ACK command mismatch')
            return ack
        ack=self.wait(applied,2,'missing action-applied ACK')
        self.action=action
        return ack
    def advance_physics(self,steps,target):
        if not self.service('/world/demo/control','gz.msgs.WorldControl','Boolean',f'pause: true multi_step: {steps}'):
            raise RuntimeError('world step rejected')
        def reached():
            with self.lock:s=dict(self.stats)
            if s['ns']>target:raise RuntimeError('simulation overshoot')
            return s['ns']==target and s['paused'] and s['iterations']==target//self.physics_ns
        self.wait(reached,10,'target simulation time')
    def receipt(self,ack,steps,target):
        def completed():
            r=self.status()
            if not r['paused']: return False
            if not r['healthy'] or not r['committed'] or r['episode_id']!=self.episode or r['action_id']!=ack['action_id'] or r['applied_ns']!=ack['applied_ns'] or r['steps']!=steps or r['first_ns']!=ack['applied_ns']+self.physics_ns or r['last_ns']!=target:
                raise RuntimeError('invalid physics execution receipt: '+str(r))
            return r
        return self.wait(completed,3,'physics receipt')
    def frame(self,target,receipt,fault=None):
        def aligned():
            with self.lock:
                if all(target in c for c in self.cache.values()):return {k:dict(c[target]) for k,c in self.cache.items()}
            return None
        samples=self.wait(aligned,8,'target lidar/odom/truth samples')
        times={k:target/1e9 for k in samples}
        if fault=='stale_lidar':
            with self.lock:
                old=[t for t in self.cache['lidar'] if t<target]
                if not old: raise RuntimeError('no old lidar available for injection')
                t=max(old);samples['lidar']=dict(self.cache['lidar'][t]);times['lidar']=t/1e9
        od=samples['odom'];x,y,yaw=od['pose'];dx,dy=3.-x,-y
        angle=math.atan2(dy,dx)-yaw
        truth=samples['truth']; tx,ty,_=truth['pose']
        return {'episode_id':self.episode-1 if fault=='cross_episode' else self.episode,'action_id':self.action,'sim_time':target/1e9,
                'sensor_times':times,'observation':{'lidar':samples['lidar'],'odom_velocity':od['velocity'],
                  'goal_relative':[min(20.,math.hypot(dx,dy))/20,math.sin(angle),math.cos(angle)]},
                'odom_pose':od['pose'],'evaluation':{'truth_pose':truth['pose'],'truth_dist':math.hypot(tx-(-8.+0.5542825+3.),ty+5.),
                    'roll':truth['roll'],'pitch':truth['pitch'],'contact':'unknown'},'ack':receipt}
    def step(self,req):
        duration=Decimal(str(req.get('dt',.2)))*Decimal(1_000_000_000)
        if not duration.is_finite() or duration<=0 or duration%self.physics_ns: raise ValueError('dt must be an exact positive multiple of physics_dt')
        if duration!=200_000_000:raise ValueError('fixed control period is 0.2 s')
        steps=int(duration/self.physics_ns);target=self.current_ns+int(duration)
        ack=self.apply(req['v'],req['w'],self.action+1,req.get('fault'))
        self.advance_physics(steps,target)
        receipt=self.receipt(ack,steps,target)
        self.current_ns=target
        return self.frame(target,receipt,req.get('fault'))
    @staticmethod
    def require_stopped(frame,receipt):
        if max(abs(x) for x in frame['observation']['odom_velocity'])>.01 or max(abs(receipt[k]) for k in ('left_measured','right_measured'))>.01:
            raise RuntimeError('residual velocity')
    @staticmethod
    def stable_pose(a,b):
        return math.hypot(a[0]-b[0],a[1]-b[1])<.001 and abs(math.atan2(math.sin(a[2]-b[2]),math.cos(a[2]-b[2])))<.001
    def stop(self):
        previous=None
        for _ in range(5):
            frame=self.step({'v':0.,'w':0.,'dt':.2})
            truth=frame['evaluation']['truth_pose']
            try:self.require_stopped(frame,frame['ack'])
            except RuntimeError:
                previous=truth;continue
            if previous and self.stable_pose(previous,truth):return frame
            previous=truth
        raise RuntimeError('stop drift did not settle')
    def close(self):
        if self.closed:return
        self.closed=True
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:self.proc.kill();self.proc.wait(timeout=3)
        if self.client:
            self.client.terminate()
            try:self.client.wait(timeout=3)
            except subprocess.TimeoutExpired:self.client.kill();self.client.wait(timeout=3)
            self.client.stdin.close();self.client.stdout.close()
        for topic in self.topics:self.node.unsubscribe(topic)
        self.node=None;gc.collect()
        if self.log:self.log.close()
        self.tmp.cleanup()

def main():
    worker=None
    try:
        for line in sys.stdin:
            try:
                req=json.loads(line);op=req['op']
                if op=='reset':
                    if worker is not None:raise RuntimeError('one episode per worker')
                    worker=Worker(req);result=worker.initial
                elif op=='step':result=worker.step(req)
                elif op=='stop':result=worker.stop()
                elif op=='close':worker.close();result=None
                else:raise ValueError('unknown operation')
                print(json.dumps({'ok':True,'result':result}),flush=True)
            except Exception as e:
                print(json.dumps({'ok':False,'error':f'{type(e).__name__}: {e}'}),flush=True)
                break
            if op=='close':break
    finally:
        if worker:worker.close()
if __name__=='__main__':main()
