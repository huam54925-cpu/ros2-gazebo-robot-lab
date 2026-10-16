"""Bounded vehicle SAC. Formal and short gradient acceptance are explicitly separate."""
import argparse
from copy import deepcopy
import json
import platform
import importlib.metadata
from pathlib import Path
import signal
import numpy as np
import torch
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback
from .checkpoints import save_checkpoint,verify_checkpoint,digest
from .envs.contracts import SCHEMA_VERSION
from .envs.local_drive_env import LocalDriveEnv

class Lifecycle(BaseCallback):
    def __init__(self,root,metadata,interval):
        super().__init__();self.root=root;self.metadata=metadata;self.interval=interval;self.saved=0;self.requested=False;self.periodic=[]
    def request(self,*_):self.requested=True
    def _on_step(self):return not self.requested
    def _on_rollout_end(self):
        if self.model.num_timesteps-self.saved>=self.interval:
            self.periodic.append(str(save_checkpoint(self.model,self.root,self.metadata)));self.saved=self.model.num_timesteps

def create_sac(env, seed, device, model_cfg):
    # SB3 mutates nested policy_kwargs (e.g. use_sde); metadata must stay immutable.
    return SAC('MlpPolicy', env, seed=seed, device=device, **deepcopy(model_cfg))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--profile',choices=['formal','acceptance'],default='formal')
    p.add_argument('--steps',type=int,required=True)
    p.add_argument('--resume-steps',type=int,default=8)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--resume',type=Path,help='Trusted vehicle checkpoint directory')
    p.add_argument('--seed',type=int,default=7)
    p.add_argument('--device',choices=['cpu','cuda','auto'],default='cpu')
    args=p.parse_args()
    if not 1<=args.steps<=1000000 or not 0<=args.resume_steps<=10000:p.error('Invalid bounded step count')
    if args.profile=='acceptance' and args.steps<32:p.error('Acceptance requires at least 32 steps to exercise updates')
    from .device import resolve_device
    device=resolve_device(args.device,torch.cuda.is_available())
    torch.set_num_threads(1)
    cfg_path=Path(__file__).parent/'configs/sac_local.json'
    cfg=json.loads(cfg_path.read_text())
    model_cfg=dict(cfg['model'])
    if args.profile=='acceptance':model_cfg.update(learning_starts=16,batch_size=32,buffer_size=5000)
    metadata={'environment':'LocalDriveEnv','schema':SCHEMA_VERSION,'profile':args.profile,'model_config':model_cfg,'seed':args.seed,
              'config_sha256':digest(cfg_path),'versions':{k:importlib.metadata.version(k) for k in ('torch','gymnasium','stable-baselines3')},'python':platform.python_version(),
              'purpose':'short vehicle gradient acceptance' if args.profile=='acceptance' else 'formal vehicle configuration'}
    args.output.mkdir(parents=True,exist_ok=True);root=args.output/'checkpoints'
    env=LocalDriveEnv();model=None;callback=None
    metadata["simulation_image"]=env.backend.image
    provenance=Path("/opt/robot-training/source-manifest.json")
    if provenance.exists():metadata["source_manifest_sha256"]=digest(provenance)
    try:
        if args.resume:
            checkpoint,manifest=verify_checkpoint(args.resume)
            if manifest.get('schema')!=SCHEMA_VERSION or manifest.get('environment')!='LocalDriveEnv' or manifest.get('profile')!=args.profile:raise ValueError('Incompatible vehicle checkpoint')
            if manifest.get('model_config')!=model_cfg:raise ValueError('Configuration mismatch')
            model=SAC.load(checkpoint/'model.zip',env=env,device=device);model.load_replay_buffer(checkpoint/'replay.pkl')
        else:model=create_sac(env,args.seed,device,model_cfg)
        callback=Lifecycle(root,metadata,16 if args.profile=='acceptance' else cfg['checkpoint']['every_environment_steps'])
        signal.signal(signal.SIGTERM,callback.request);signal.signal(signal.SIGINT,callback.request)
        initial_steps=model.num_timesteps;initial_updates=model._n_updates
        actor_before=[p.detach().clone() for p in model.actor.parameters()]
        model.learn(args.steps,reset_num_timesteps=False,callback=callback)
        env.backend.stop()
        checkpoint=save_checkpoint(model,root,metadata)
        verified,manifest=verify_checkpoint(root)
        probe=np.zeros(295,dtype=np.float32);pred=model.predict(probe,deterministic=True)[0]
        replay_size=model.replay_buffer.size()
        reloaded=SAC.load(verified/'model.zip',env=env,device=device);reloaded.load_replay_buffer(verified/'replay.pkl')
        np.testing.assert_allclose(pred,reloaded.predict(probe,deterministic=True)[0],atol=1e-7,rtol=0)
        assert reloaded.replay_buffer.size()==replay_size
        updates=model._n_updates-initial_updates
        actor_delta=max(float((a-b.detach()).abs().max()) for a,b in zip(actor_before,model.actor.parameters()))
        if args.profile=='acceptance' and not callback.requested:
            assert updates>0 and actor_delta>0,'No actual gradient update'
        resumed_before=reloaded.num_timesteps
        if args.resume_steps and not callback.requested:
            # Loaded learner starts a new episode, never reuses a saved sim state.
            reloaded.learn(args.resume_steps,reset_num_timesteps=False,callback=callback)
            assert reloaded.num_timesteps==resumed_before+args.resume_steps or callback.requested
            env.backend.stop();checkpoint=save_checkpoint(reloaded,root,metadata)
        report={'status':'stopped' if callback.requested else 'passed','profile':args.profile,'schema':SCHEMA_VERSION,
                'learning_starts':model_cfg['learning_starts'],'initial_steps':initial_steps,'sampled_steps':model.num_timesteps-initial_steps,
                'gradient_updates':updates,'actor_max_parameter_change':actor_delta,'replay_before_reload':replay_size,
                'resumed_steps':reloaded.num_timesteps-resumed_before,'final_replay_size':reloaded.replay_buffer.size(),
                'checkpoint':str(checkpoint),'periodic_checkpoints':callback.periodic,'device':device,'simulation_rendering':'GPU EGL' if env.backend.gpu else 'Mesa llvmpipe EGL','renderer':getattr(env.backend,'renderer',[])}
        (args.output/'vehicle-training.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2),flush=True)
    finally:env.close()
if __name__=='__main__':main()
