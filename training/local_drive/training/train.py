"""Bounded SAC toolchain acceptance. This entry does not drive Gazebo."""
import argparse
import json
from pathlib import Path
import signal
import time
import gymnasium as gym
import numpy as np
import torch
import stable_baselines3 as sb3
from stable_baselines3.common.callbacks import BaseCallback
from training.checkpoints import save_checkpoint, verify_checkpoint, digest

class StopFlag(BaseCallback):
    def __init__(self):
        super().__init__(); self.requested=False
    def _on_step(self): return not self.requested
    def request(self,*_): self.requested=True

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend',choices=['pendulum-smoke','gazebo'],default='pendulum-smoke')
    p.add_argument('--device',choices=['cpu','cuda','auto'],default='cpu')
    p.add_argument('--steps',type=int,default=128)
    p.add_argument('--output',type=Path,default=Path('/runs'))
    a=p.parse_args()
    if a.backend=='gazebo': p.error('Gazebo training is gated: action ACK + synchronized step/reset backend not implemented')
    if not 32 <= a.steps <= 10000: p.error('Smoke steps must be in [32,10000]')
    from training.device import resolve_device
    try: device=resolve_device(a.device, torch.cuda.is_available())
    except ValueError as exc: p.error(str(exc))
    torch.set_num_threads(1); a.output.mkdir(parents=True,exist_ok=True)
    cfg_path=Path(__file__).parent/'configs/sac_local.json'
    cfg=json.loads(cfg_path.read_text())
    env=gym.make('Pendulum-v1'); stop=StopFlag()
    signal.signal(signal.SIGTERM,stop.request); signal.signal(signal.SIGINT,stop.request)
    kwargs=dict(cfg['model']); kwargs.update(buffer_size=2048,learning_starts=16,batch_size=16,policy_kwargs={'net_arch':[32,32]})
    model=sb3.SAC(cfg['policy'],env,seed=cfg['seed'],device=device,**kwargs)
    metadata={'purpose':'Pendulum toolchain smoke ONLY; not a robot policy',
              'environment':'Pendulum-v1','config_sha256':digest(cfg_path),
              'effective_model':kwargs,'seed':cfg['seed'],'device':device,'requested_device':a.device,
              'versions':{'sb3':sb3.__version__,'torch':torch.__version__,'gymnasium':gym.__version__}}
    (a.output/'ready.json').write_text(json.dumps({'ready':True,'environment':'Pendulum-v1'}))
    started=time.monotonic()
    try:
        model.learn(total_timesteps=a.steps,callback=stop)
        path=save_checkpoint(model,a.output/'checkpoints',metadata)
        verified,manifest=verify_checkpoint(a.output/'checkpoints')
        obs,_=env.reset(seed=42)
        before=model.predict(obs,deterministic=True)[0]
        resumed=sb3.SAC.load(verified/'model.zip',env=env,device=device)
        resumed.load_replay_buffer(verified/'replay.pkl')
        after=resumed.predict(obs,deterministic=True)[0]
        np.testing.assert_allclose(before,after,rtol=0,atol=1e-7)
        assert resumed.replay_buffer.size()==model.replay_buffer.size()
        initial_steps=resumed.num_timesteps
        if not stop.requested:
            resumed.learn(total_timesteps=32,reset_num_timesteps=False,callback=stop)
            if not stop.requested: assert resumed.num_timesteps==initial_steps+32
            path=save_checkpoint(resumed,a.output/'checkpoints',metadata)
        report=dict(device=device,requested_device=a.device,status='stopped' if stop.requested else 'passed',purpose=metadata['purpose'],
                    initial_steps=initial_steps,resumed_steps=resumed.num_timesteps,
                    replay_size=resumed.replay_buffer.size(),deterministic_reload_max_error=float(np.max(np.abs(before-after))),
                    elapsed_wall_seconds=time.monotonic()-started,checkpoint=str(path),versions=metadata['versions'])
        (a.output/'trainer-smoke.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
    finally: env.close()

if __name__=='__main__':main()
