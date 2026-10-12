"""Real Gazebo acceptance. Runs inside trainer with access to its host Docker daemon."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import numpy as np
import torch
from stable_baselines3 import SAC
from stable_baselines3.common.env_checker import check_env
from gymnasium.utils.env_checker import check_env as gym_check_env
from training.envs.gazebo_backend import GazeboBackendImpl
from training.envs.local_drive_env import LocalDriveEnv
from training.envs.contracts import InfrastructureError


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--stage',choices=['backend','env','all'],default='all');a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=True);report={};torch.set_num_threads(1)
    if a.stage in ('backend','all'):
        backend=GazeboBackendImpl();runs=[]
        try:
            for trial in range(3):
                frames=[backend.reset(seed=7)]
                for v,w in ((.1,0),(.2,.4),(-.1,0),(0,0)):
                    frames.append(backend.advance(v,w))
                backend.stop()
                runs.append(frames)
                print(f'cold episode {trial+1}: passed',flush=True)
            for frames in runs[1:]:
                for x,y in zip(runs[0],frames):
                    assert x.sensor_times==y.sensor_times
                    np.testing.assert_allclose(x.odom_pose,y.odom_pose,atol=1e-10,rtol=0)
                    np.testing.assert_allclose(x.evaluation['truth_pose'],y.evaluation['truth_pose'],atol=1e-9,rtol=0)
                    np.testing.assert_allclose(x.observation['lidar']['ranges'],y.observation['lidar']['ranges'],atol=1e-6,rtol=0)
            report['backend']={'cold_episodes':3,'action_steps':12,'strict_tolerance_s':1e-6,'repeatability':'passed'}
            (a.output/'backend-traces.json').write_text(json.dumps([[asdict(f) for f in run] for run in runs],indent=2))
        finally:backend.close()
    if a.stage in ('env','all'):
        env=LocalDriveEnv();records=[]
        try:
            check_env(env,warn=True)
            print('SB3 check_env: passed',flush=True)
            gym_check_env(env,skip_render_check=True)
            print('Gymnasium check_env: passed',flush=True)
            rng=np.random.default_rng(11)
            for ep in range(3):
                obs,info=env.reset(seed=23)
                assert np.all(obs[-20:]==0)
                assert max(abs(x) for x in env.last_frame.observation['odom_velocity'])<.01
                for _ in range(12):
                    action=rng.uniform(-1,1,2).astype(np.float32)
                    obs,reward,term,trunc,info=env.step(action)
                    np.testing.assert_allclose(obs[-5:-3],action)
                    assert obs.shape==(295,) and obs.dtype==np.float32 and np.isfinite(obs).all()
                    records.append({'action':action.tolist(),'obs':obs.tolist(),'reward':reward,'terminated':term,'truncated':trunc,'frame':asdict(env.last_frame)})
                    if term or trunc:break
            report['random_audit']={'episodes':3,'transitions':len(records),'passed':True}
            (a.output/'random-audit.json').write_text(json.dumps(records))
            model=SAC('MlpPolicy',env,seed=17,learning_starts=5000,buffer_size=256,policy_kwargs={'net_arch':[32,32]},device='cpu')
            failures=[]
            for fault in ('missing_ack','stale_lidar','cross_episode'):
                model.set_env(env,force_reset=True);model.learn(2,reset_num_timesteps=False)
                before=model.replay_buffer.size();name=env.backend.name
                env.backend.fault_next=fault
                original_rpc=env.backend._rpc
                # Also make stop fail during ACK failure: the sampling error must survive.
                if fault=='missing_ack':
                    def failing_stop(op,**kwargs):
                        if op=='stop':raise InfrastructureError('injected cleanup stop failure')
                        return original_rpc(op,**kwargs)
                    env.backend._rpc=failing_stop
                try:model.learn(1,reset_num_timesteps=False)
                except InfrastructureError as exc:
                    assert model.replay_buffer.size()==before
                    assert env.backend.proc is None
                    if fault=='missing_ack':assert 'action-applied ACK' in str(exc)
                    failures.append({'fault':fault,'error':str(exc),'notes':getattr(exc,'__notes__',[]),'replay_before':before,'replay_after':model.replay_buffer.size(),'container':name})
                else:raise AssertionError('fault did not fail: '+fault)
                finally:env.backend._rpc=original_rpc
                env.close();env.close() # idempotent after failure
                obs,_=env.reset(seed=17);assert np.all(obs[-20:]==0)
                print(f'fault {fault}: discarded, destroyed, rebuilt',flush=True)
            report['faults']=failures
        finally:env.close()
    report['status']='passed'
    (a.output/'acceptance.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2),flush=True)
if __name__=='__main__':main()
