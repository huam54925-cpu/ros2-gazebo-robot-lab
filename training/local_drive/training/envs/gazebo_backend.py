"""One private container per episode, with synchronous worker RPC and fail-closed cleanup."""
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import tempfile
import threading
import uuid
from .contracts import Frame, InfrastructureError, validate_transition

class GazeboBackendImpl:
    def __init__(self, *, image=None, source_root=None, plugin_dir=None, gpu=None, timeout=60):
        self.image=image or os.environ.get('LOCAL_DRIVE_SIM_IMAGE','local-drive-sync-sim:20261012-v2')
        source_root=source_root or os.environ.get('LOCAL_DRIVE_REPO')
        self.source_root=Path(source_root).resolve() if source_root else None
        plugin_dir=plugin_dir or os.environ.get('LOCAL_DRIVE_PLUGIN_DIR')
        self.plugin_dir=Path(plugin_dir).resolve() if plugin_dir else None
        self.gpu=(os.environ.get("LOCAL_DRIVE_SIM_GPU","1")=="1") if gpu is None else bool(gpu)
        self.timeout=timeout;self.episode_id=0;self.prev_frame=None
        self.proc=None;self.name=None;self.stderr=None;self.thread=None;self.responses=None
        self.cleanup_errors=[];self.fault_next=None
    def _reader(self,proc,responses):
        try:
            for line in proc.stdout:
                try:responses.put(json.loads(line))
                except json.JSONDecodeError:responses.put({'ok':False,'error':'Invalid worker protocol: '+line[:500]})
        finally:responses.put({'ok':False,'error':'Worker disconnected'})
    def _rpc(self,op,**args):
        if self.proc is None:raise InfrastructureError('Backend is not running; reset required')
        try:
            self.proc.stdin.write(json.dumps({'op':op,**args})+'\n');self.proc.stdin.flush()
            response=self.responses.get(timeout=self.timeout)
            if not response.get('ok'):raise InfrastructureError(response.get('error','worker failure'))
            return response['result']
        except (OSError,ValueError,queue.Empty) as e:raise InfrastructureError('Worker RPC failed: '+str(e)) from e
    def reset(self, *, seed:int, scenario='indoor_fixed'):
        if scenario!='indoor_fixed':raise ValueError('Only indoor_fixed is implemented')
        if not isinstance(seed,int) or not 0<=seed<2**31:raise ValueError('seed must be nonnegative int32')
        self.close()
        if self.name:raise InfrastructureError("Prior container destruction failed; refusing rebuild")
        self.cleanup_errors=[];self.episode_id+=1;self.prev_frame=None
        self.name='local-drive-'+uuid.uuid4().hex[:16]
        self.stderr=tempfile.TemporaryFile(mode='w+')
        args=['docker','run','--pull','never','--rm','-i','--name',self.name,'--shm-size=1g',
              '-e','GZ_PARTITION='+self.name]
        run_id=os.environ.get('LOCAL_DRIVE_RUN_ID')
        if run_id:
            if len(run_id)!=32 or any(c not in '0123456789abcdef' for c in run_id):
                raise ValueError('Invalid supervisor run identity')
            args+=['--label','org.local-drive.run='+run_id,'--label','org.local-drive.role=sim']
        if self.source_root:args+=['-v',str(self.source_root)+':/src:ro']
        if self.plugin_dir:args+=['-v',str(self.plugin_dir)+':/sync:ro']
        if self.gpu:args+=['--gpus','all']
        else:args+=['-e','LIBGL_ALWAYS_SOFTWARE=1','-e','GALLIUM_DRIVER=llvmpipe']
        args += [self.image,'python3','/src/training/local_drive/sim_sync/worker.py']
        try:
            self.proc=subprocess.Popen(args,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.stderr,text=True,bufsize=1)
            self.responses=queue.Queue();self.thread=threading.Thread(target=self._reader,args=(self.proc,self.responses),daemon=True);self.thread.start()
            frame=Frame(**self._rpc('reset',seed=seed,episode_id=self.episode_id))
            self._validate_frame(frame)
            if frame.episode_id!=self.episode_id or frame.action_id!=0 or abs(frame.sim_time-.6)>1e-6:raise InfrastructureError('Invalid reset identity/time')
            self._validate_receipt(frame,0.,0.,0.,600)
            if max(abs(v) for v in frame.observation['odom_velocity'])>.01:raise InfrastructureError('Residual reset velocity')
            self.renderer=frame.evaluation.get("renderer",[])
            self.prev_frame=frame;return frame
        except Exception as exc:self._fail(exc)
    @staticmethod
    def _validate_frame(frame):
        try:
            if not math.isfinite(frame.sim_time):raise ValueError('invalid sim time')
            if set(frame.observation)!={'lidar','odom_velocity','goal_relative'}:raise ValueError('unexpected raw observation fields')
            if len(frame.odom_pose)!=3 or len(frame.observation['odom_velocity'])!=2 or len(frame.observation['goal_relative'])!=3:raise ValueError('invalid odom/goal shape')
            if not all(math.isfinite(x) for x in [*frame.odom_pose,*frame.observation['odom_velocity'],*frame.observation['goal_relative']]):raise ValueError('nonfinite odom/goal')
            lidar=frame.observation['lidar']
            if len(lidar['ranges'])!=360 or not 0<=lidar['range_min']<lidar['range_max']:raise ValueError('invalid lidar geometry')
            if not math.isfinite(frame.evaluation['truth_dist']) or frame.evaluation['truth_dist']<0:raise ValueError('invalid truth distance')
            for key in ('roll','pitch'):
                if not math.isfinite(frame.evaluation[key]):raise ValueError('invalid attitude')
            if len(frame.evaluation['truth_pose'])!=3 or not all(math.isfinite(x) for x in frame.evaluation['truth_pose']):raise ValueError('invalid truth pose')
            for key in ('lidar','odom','truth'):
                if not math.isfinite(frame.sensor_times[key]) or abs(frame.sensor_times[key]-frame.sim_time)>1e-6:raise ValueError('stale '+key)
        except (KeyError,TypeError,ValueError) as e:raise InfrastructureError('Invalid frame: '+str(e)) from e
    @staticmethod
    def _validate_receipt(frame,v,w,start,steps):
        a=frame.ack
        try:
            if not (a['healthy'] and a['committed'] and a['paused']) or a['episode_id']!=frame.episode_id or a['action_id']!=frame.action_id:raise ValueError('ACK identity/state')
            if a['applied_ns']!=round(start*1e9) or a['steps']!=steps or a['first_ns']!=round(start*1e9)+1_000_000 or a['last_ns']!=round(frame.sim_time*1e9):raise ValueError('ACK execution interval')
            for key,expected in [('v',v),('w',w),('left_cmd',(v-w*1.25/2)/.3),('right_cmd',(v+w*1.25/2)/.3)]:
                if not math.isfinite(a[key]) or abs(a[key]-expected)>1e-10:raise ValueError('ACK actuator mismatch')
        except (KeyError,ValueError,TypeError) as e:raise InfrastructureError('Invalid ACK: '+str(e)) from e
    def advance(self,v,w,dt=.2):
        if not all(math.isfinite(x) for x in (v,w,dt)) or not -.2-1e-9<=v<=.6+1e-9 or abs(w)>.8+1e-9 or dt!=.2:raise ValueError('Invalid fixed-period action')
        if self.prev_frame is None:raise InfrastructureError('reset required')
        try:
            fault=self.fault_next;self.fault_next=None
            frame=Frame(**self._rpc('step',v=v,w=w,dt=dt,fault=fault))
            validate_transition(self.prev_frame,frame,dt=dt)
            self._validate_frame(frame)
            self._validate_receipt(frame,v,w,self.prev_frame.sim_time,200)
            self.prev_frame=frame;return frame
        except Exception as exc:self._fail(exc)
    def _fail(self,exc):
        # Preserve the original failure even when stopping or destruction fails.
        original=exc if isinstance(exc,InfrastructureError) else InfrastructureError(str(exc))
        try:self.close()
        except Exception as cleanup:self.cleanup_errors.append(repr(cleanup))
        for error in self.cleanup_errors:original.add_note('cleanup: '+error)
        raise original from (exc if original is not exc else None)
    def stop(self):
        if self.proc is None:return
        try:self._rpc('stop')
        except Exception:
            self._destroy();raise
        finally:self.prev_frame=None # stop advances physics; a new reset is mandatory
    def _destroy(self):
        name=self.name;proc=self.proc
        self.proc=None;self.prev_frame=None
        if name:
            try:
                r=subprocess.run(['docker','rm','-f',name],capture_output=True,text=True,timeout=15)
                if r.returncode and 'No such container' not in r.stderr:self.cleanup_errors.append(r.stderr.strip())
                else:self.name=None
            except Exception as e:self.cleanup_errors.append(repr(e))
        if proc:
            try:
                if proc.stdin:proc.stdin.close()
                proc.wait(timeout=5)
            except Exception:
                try:proc.kill();proc.wait(timeout=5)
                except Exception as e:self.cleanup_errors.append(repr(e))
            finally:
                if proc.stdout:proc.stdout.close()
        if self.thread:self.thread.join(timeout=2);self.thread=None
        if self.stderr:self.stderr.close();self.stderr=None
    def close(self):
        if self.proc is None:
            if self.name:self._destroy()
            return
        try:self._rpc('stop')
        except Exception as e:self.cleanup_errors.append(str(e))
        finally:self._destroy()
