"""Start the accepted scheduler with explicit limits on learner and simulator RAM."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

PROJECT=Path('/home/user111/AI-Training/offline-bundle-20261017/source/training/local_drive')
RUN=Path('/home/user111/AI-Training/runs/sac-longrun-20261017')
CONTROL=Path('/home/user111/AI-Training/launchers/memory-bounded-20261017')
TRAINER='local-drive-sync-trainer:20261016-longrun'
SIM='local-drive-sync-sim:20261012-v2'
os.environ.update(DOCKER_CONTEXT='wsl-native',TRAINER_IMAGE=TRAINER,LOCAL_DRIVE_SIM_IMAGE=SIM)
os.environ.pop('DOCKER_HOST',None)

def docker(*args):
    return subprocess.check_output(['docker',*args],text=True,timeout=60).strip()

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--hours',type=float,default=3)
    parser.add_argument('--worker',action='store_true')
    args=parser.parse_args()
    if not 0<args.hours<=3:parser.error('hours must be between 0 and 3')
    if args.worker:
        limits=json.loads((CONTROL/'limits.json').read_text())
        sys.path.insert(0,str(PROJECT/'scripts'))
        import long_train
        original=long_train.common_args
        def bounded(*a,**kw):
            return original(*a,**kw)+[
                '--memory=6g','--memory-swap=6g',
                '-v',str(CONTROL/'docker')+':/opt/local-drive-resource-guard/docker:ro',
                '-e','PATH=/opt/local-drive-resource-guard:'+limits['image_path'],
                '-e','LD_REAL_DOCKER='+limits['real_docker']]
        long_train.common_args=bounded
        long_train.main(['--run-dir',str(RUN),'--hours',str(args.hours),
            '--chunk-steps','1000','--eval-every','5000','--eval-seed','7'])
        return
    RUN.mkdir(parents=True,exist_ok=True)
    CONTROL.mkdir(parents=True,exist_ok=True)
    if (RUN/'STOP').exists():raise RuntimeError('STOP exists; confirm stopped and remove STOP before resuming')
    pidfile=RUN/'manager.pid'
    if pidfile.exists():
        pid=int(pidfile.read_text())
        cmdline=Path(f'/proc/{pid}/cmdline')
        if cmdline.exists() and b'memory-bounded-20261017' in cmdline.read_bytes():
            raise RuntimeError(f'Manager already running with PID {pid}')
    if shutil.disk_usage(RUN).free<60*1024**3:raise RuntimeError('Less than 60 GiB free in WSL')
    mem=dict(line.split(':',1) for line in Path('/proc/meminfo').read_text().splitlines())
    if int(mem['MemAvailable'].split()[0])<10*1024**2:raise RuntimeError('Need at least 10 GiB available WSL RAM before launch')
    info=json.loads(docker('image','inspect',TRAINER))[0]
    image_path=next(v[5:] for v in info['Config']['Env'] if v.startswith('PATH='))
    real_docker=docker('run','--rm','--pull','never','--network','none','--memory=512m','--memory-swap=512m','--entrypoint','sh',TRAINER,'-c','command -v docker')
    if not real_docker.startswith('/'):raise RuntimeError('Docker executable path unresolved')
    guard='#!/bin/sh\nset -eu\nif [ "${1:-}" = run ]; then\n  shift\n  exec "$LD_REAL_DOCKER" run --memory=2g --memory-swap=2g "$@"\nfi\nexec "$LD_REAL_DOCKER" "$@"\n'
    (CONTROL/'docker').write_text(guard)
    (CONTROL/'docker').chmod(0o755)
    limits={'trainer_memory_bytes':6*1024**3,'sim_memory_bytes':2*1024**3,'container_swap_extra_bytes':0,
        'image_path':image_path,'real_docker':real_docker,'project':str(PROJECT),'run':str(RUN),
        'note':'Container limits do not cap Windows, Docker daemon or other WSL workloads. Release source and image files unchanged.'}
    (CONTROL/'limits.json').write_text(json.dumps(limits,indent=2)+'\n')
    target=CONTROL/'launch-memory-bounded-20261017.py'
    if Path(__file__).resolve()!=target:shutil.copy2(__file__,target)
    # Verify the bind-mounted CLI guard before starting a long run.
    actual=docker('run','--rm','--pull','never','--network','none','--memory=512m','--memory-swap=512m',
        '-v',str(CONTROL/'docker')+':/opt/local-drive-resource-guard/docker:ro',
        '-e','PATH=/opt/local-drive-resource-guard:'+image_path,
        '-e','LD_REAL_DOCKER='+real_docker,'--entrypoint','sh',TRAINER,'-c','command -v docker; docker --version')
    if not actual.startswith('/opt/local-drive-resource-guard/docker\n'):raise RuntimeError('CLI guard was not selected')
    preflight=subprocess.run([sys.executable,str(PROJECT/'scripts/preflight_longrun.py'),'--data-dir',str(RUN)],capture_output=True,text=True,timeout=180)
    (RUN/'launch-preflight.stdout.txt').write_text(preflight.stdout)
    (RUN/'launch-preflight.stderr.txt').write_text(preflight.stderr)
    preflight.check_returncode()
    with (RUN/'manager-console.log').open('a') as log:
        proc=subprocess.Popen([sys.executable,'-u',str(target),'--worker','--hours',str(args.hours)],
            stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,cwd=PROJECT)
    pidfile.write_text(str(proc.pid)+'\n')
    (RUN/'resource-limits.json').write_text(json.dumps(limits,indent=2)+'\n')
    print(json.dumps({'status':'launched','pid':proc.pid,'hours':args.hours,'run':str(RUN),'limits':limits},indent=2),flush=True)

if __name__=='__main__':main()
