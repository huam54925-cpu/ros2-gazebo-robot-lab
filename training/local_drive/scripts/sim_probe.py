"""Bounded real-simulation acceptance, NOT a training step implementation."""
import json
import math
import threading
import subprocess
import time
from gz.transport import Node
from gz.msgs.laserscan_pb2 import LaserScan
from gz.msgs.odometry_pb2 import Odometry
from gz.msgs.pose_v_pb2 import Pose_V
from gz.msgs.world_stats_pb2 import WorldStatistics
from gz.msgs.twist_pb2 import Twist

node=Node(); lock=threading.Lock(); data={}
def stamp(t):return t.sec+t.nsec*1e-9
def put(key,value):
    with lock:data[key]=value

def scan(m):
    ranges=list(m.ranges)
    put('scan',dict(t=stamp(m.header.stamp),count=len(ranges),
        finite=sum(math.isfinite(x) for x in ranges),
        valid=sum(math.isfinite(x) and m.range_min <= x <= m.range_max for x in ranges),
        minimum=min((x for x in ranges if math.isfinite(x)),default=None)))
def truth(m):
    p=next((p for p in m.pose if p.name=='vehicle'),None)
    if p:put('truth',dict(t=stamp(m.header.stamp),x=p.position.x,y=p.position.y,z=p.position.z))
def odom(m):put('odom',dict(t=stamp(m.header.stamp),v=m.twist.linear.x,w=m.twist.angular.z))
def stats(m):put('stats',dict(t=stamp(m.sim_time),paused=m.paused,iterations=m.iterations))

def snapshot():
    with lock:return dict(data)
def wait(predicate,timeout=25):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        d=snapshot()
        if predicate(d):return d
        time.sleep(.02)
    raise TimeoutError(json.dumps(snapshot()))
def control(pause):
    # Native CLI avoids observed intermittent Python request timeout while callbacks run.
    # Diagnostic only: this is explicitly NOT the proposed training control path.
    result=subprocess.run(['gz','service','-s','/world/demo/control',
        '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean',
        '--timeout','5000','--req','pause: '+str(pause).lower()],
        capture_output=True,text=True,timeout=10,check=True)
    if 'data: true' not in result.stdout:raise RuntimeError('World control rejected: '+result.stdout)

for msg,topic,cb in [(LaserScan,'/scan',scan),(Odometry,'/model/vehicle/odometry',odom),
                     (Pose_V,'/world/demo/dynamic_pose/info',truth),(WorldStatistics,'/world/demo/stats',stats)]:
    if not node.subscribe(msg,topic,cb):raise RuntimeError('Subscription failed: '+topic)
publisher=node.advertise('/model/vehicle/cmd_vel',Twist)
def command(v):
    msg=Twist();msg.linear.x=v
    if not publisher.publish(msg):raise RuntimeError('Publish failed')
report={'purpose':'Real Gazebo smoke; asynchronous diagnostic only, not synchronized RL sampling'}
try:
    initial=wait(lambda d:'stats' in d,timeout=45)
    if not initial['stats']['paused']:raise AssertionError('Expected startup paused')
    report['initial']=initial
    time.sleep(.25)
    control(False)
    settled=wait(lambda d:all(k in d for k in ('scan','truth','odom','stats')) and d['scan']['t']>.5 and d['truth']['t']>.5)
    if settled['scan']['count'] != 360 or settled['scan']['valid'] < 1:raise AssertionError('Invalid LiDAR')
    report['settled']=settled
    origin=settled['truth']
    deadline=time.monotonic()+15
    while time.monotonic()<deadline:
        command(.1); time.sleep(.05)
        state=snapshot()
        if state['truth']['t']-origin['t']>=1.0:break
    else:raise TimeoutError('Motion did not advance')
    moved=snapshot();report['moved']=moved
    distance=math.hypot(moved['truth']['x']-origin['x'],moved['truth']['y']-origin['y'])
    if not .02 < distance < .35:raise AssertionError(f'Unexpected displacement: {distance}')
    report['truth_displacement_m']=distance
    for _ in range(10):command(0.);time.sleep(.05)
    stopped=wait(lambda d:d['odom']['t']>moved['odom']['t'] and abs(d['odom']['v'])<.01 and abs(d['odom']['w'])<.01)
    stop_truth=stopped['truth']
    stable=wait(lambda d:d['truth']['t']>=stop_truth['t']+.3)
    drift=math.hypot(stable['truth']['x']-stop_truth['x'],stable['truth']['y']-stop_truth['y'])
    if drift>.01:raise AssertionError(f'Stop drift {drift}')
    report['stop_drift_m']=drift
    control(True)
    paused=wait(lambda d:d['stats']['paused'])
    time.sleep(.35)
    final=snapshot()
    if abs(final['stats']['t']-paused['stats']['t'])>1e-9:raise AssertionError('Paused clock advanced')
    report['final']=final;report['status']='passed'
except Exception as exc:
    report['status']='failed';report['error']=repr(exc);report['last']=snapshot()
    raise
finally:
    try:
        for _ in range(3):command(0.);time.sleep(.05)
        control(True)
    except Exception as exc:report['cleanup_error']=repr(exc)
    # Stop native callback threads before Python module teardown.
    for topic in ('/scan','/model/vehicle/odometry','/world/demo/dynamic_pose/info','/world/demo/stats'):
        node.unsubscribe(topic)
    publisher=None
    node=None
    import gc
    gc.collect()
    if 'cleanup_error' in report: report['status']='failed'
    print(json.dumps(report,indent=2),flush=True)
    if 'cleanup_error' in report: raise RuntimeError(report['cleanup_error'])
