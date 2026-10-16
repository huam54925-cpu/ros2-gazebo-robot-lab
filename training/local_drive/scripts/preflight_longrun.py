#!/usr/bin/env python3
"""Read-only prerequisite check. Never installs drivers, ROS, CUDA, or pip packages."""
import argparse,json,os,shutil,subprocess
from pathlib import Path
from long_train import resolve_context,inspect_image,check_image_sources,docker

def main():
 p=argparse.ArgumentParser();p.add_argument('--data-dir',type=Path,required=True);a=p.parse_args()
 root=a.data_dir.expanduser().resolve()
 if not root.is_dir():p.error('Create the data directory on the intended data drive first')
 context,socket=resolve_context()
 trainer=inspect_image(os.environ.get('TRAINER_IMAGE','local-drive-sync-trainer:20261016-longrun'))
 sim=inspect_image(os.environ.get('LOCAL_DRIVE_SIM_IMAGE','local-drive-sync-sim:20261012-v2'))
 check_image_sources(trainer)
 probe="import json,torch,gymnasium,stable_baselines3; assert torch.cuda.is_available(); x=torch.ones((32,32),device='cuda',requires_grad=True); (x@x).sum().backward(); print(json.dumps(dict(torch=torch.__version__,gymnasium=gymnasium.__version__,sb3=stable_baselines3.__version__,gpu=torch.cuda.get_device_name(0),cuda_backward=True)))"
 result=json.loads(docker(['run','--rm','--pull','never','--network','none','--gpus','all','--entrypoint','python',trainer,'-c',probe],timeout=120).stdout)
 info={'context':context,'socket':socket,'docker_root':docker(['info','--format','{{.DockerRootDir}}']).stdout.strip(),
  'data_dir':str(root),'free_gb':round(shutil.disk_usage(root).free/1024**3,2),'trainer':trainer,'sim':sim,'cuda_probe':result,
  'storage_note':'WSL mount / does not reveal the Windows VHD location; check DistributionName/BasePath in PowerShell. Docker containerd root also needs checking.'}
 print(json.dumps(info,indent=2))
 if info['free_gb']<10:raise SystemExit('Need at least 10 GiB free; recommend >=60 GiB for model/replay checkpoints')
if __name__=='__main__':main()
