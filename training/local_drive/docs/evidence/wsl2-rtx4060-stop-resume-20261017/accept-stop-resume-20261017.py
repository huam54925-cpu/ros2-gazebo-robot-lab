"""Target-machine lifecycle acceptance; leaves release sources unchanged."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

PROJECT = Path('/home/user111/AI-Training/offline-bundle-20261017/source/training/local_drive')
ROOT = Path('/home/user111/AI-Training/diagnostics/stop-resume-20261017')
EXPORT = Path('/mnt/f/AI-Training/diagnostics/stop-resume-20261017')
ENV = dict(os.environ, DOCKER_CONTEXT='wsl-native', TRAINER_IMAGE='local-drive-sync-trainer:20261016-longrun', LOCAL_DRIVE_SIM_IMAGE='local-drive-sync-sim:20261012-v2')
ENV.pop('DOCKER_HOST', None)

def read(p):
    return json.loads(p.read_text())

def save(p, data):
    p.write_text(json.dumps(data, indent=2)+'\n')

def digest(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024), b''): h.update(b)
    return h.hexdigest()

def docker(*args):
    return subprocess.check_output(['docker',*args],env=ENV,text=True,timeout=45).strip()

def latest(run):
    pointer=run/'checkpoints/latest.txt'
    if not pointer.exists():return None,None
    leaf=run/'checkpoints'/pointer.read_text().strip()
    return leaf,read(leaf/'manifest.json')

def verify(run):
    leaf,m=latest(run)
    assert m and m['profile']=='formal' and m['num_timesteps']>0
    for name in ('model.zip','replay.pkl'):
        assert digest(leaf/name)==m['files'][name], name
    return leaf,m

def export():
    EXPORT.mkdir(parents=True,exist_ok=True)
    for p in ROOT.rglob('*'):
        if p.is_file() and p.suffix not in ('.pkl','.zip') and p.name!='.lock':
            target=EXPORT/p.relative_to(ROOT)
            target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(p,target)

def phase(run, number, target, method):
    cmd=[sys.executable,'-u',str(PROJECT/'scripts/long_train.py'),'--run-dir',str(run),'--hours','0.2','--chunk-steps','16','--eval-every','0','--max-failures','1']
    save(ROOT/f'phase-{number}-command.json', {'argv':cmd,'stop_method':method,'stop_after_published_steps':target})
    requested=False
    with (ROOT/f'phase-{number}-console.log').open('w') as log:
        proc=subprocess.Popen(cmd,env=ENV,stdout=log,stderr=subprocess.STDOUT)
        started=time.monotonic(); next_status=0
        try:
            while proc.poll() is None:
                elapsed=time.monotonic()-started
                leaf,m=latest(run)
                if not requested and m and m['num_timesteps']>=target:
                    if method=='STOP':(run/'STOP').touch()
                    else:proc.send_signal(signal.SIGTERM)
                    requested=True
                    save(ROOT/f'phase-{number}-stop-request.json',{'manager_pid':proc.pid,'method':method,'published_steps':m['num_timesteps'],'elapsed_seconds':elapsed})
                    print(f'PHASE {number}: requested {method} at published step {m["num_timesteps"]}',flush=True)
                if elapsed>=next_status:
                    state=read(run/'long-run-state.json') if (run/'long-run-state.json').exists() else {}
                    print(f'PHASE {number}: elapsed={elapsed:.0f}s state={state}',flush=True)
                    export();next_status=elapsed+30
                if elapsed>950:raise TimeoutError('Acceptance phase exceeded bounded deadline')
                time.sleep(.25)
        finally:
            if proc.poll() is None:
                (run/'STOP').touch()
                proc.send_signal(signal.SIGTERM)
                proc.wait(timeout=200)
    state=read(run/'long-run-state.json')
    save(ROOT/f'phase-{number}-state.json',state)
    assert requested, 'Process exited before requested stop'
    assert proc.returncode==0 and state['phase']=='stopped', (proc.returncode,state)
    leaf,m=verify(run)
    identity=read(run/'run-identity.json')
    remaining=docker('ps','-aq','--filter','label=org.local-drive.run='+identity['run_id']).split()
    assert not remaining,remaining
    reports=[read(p) for p in sorted((run/'reports').glob('*.json'))]
    record={'phase':number,'manager_exit_code':proc.returncode,'state':state,'checkpoint':str(leaf),'manifest':m,'owned_containers_after_exit':remaining,'reports':reports,'elapsed_seconds':time.monotonic()-started}
    save(ROOT/f'phase-{number}-verification.json',record)
    export()
    print(f'PHASE {number}: VERIFIED stopped, steps={m["num_timesteps"]}, owned containers=0',flush=True)
    return record,identity

def main():
    ROOT.mkdir(parents=True,exist_ok=False)
    run=ROOT/'run'
    save(ROOT/'scope.json',{'source_revision':'c0ffb4d4ff3d268eaa456a89939f0cbcfc4045d3','reviewed_branch':'feature/long-train-offline-20261016','reviewed_head':'272a85a06146570473288455c5d429048c537f7f','profile':'formal','chunk_steps':16,'evaluation_enabled':False,'purpose':'stop and cross-process resume acceptance, not navigation quality or long-run stability'})
    probe=subprocess.run([sys.executable,str(PROJECT/'scripts/preflight_longrun.py'),'--data-dir',str(ROOT)],env=ENV,text=True,capture_output=True,timeout=150)
    (ROOT/'preflight.stdout.txt').write_text(probe.stdout)
    (ROOT/'preflight.stderr.txt').write_text(probe.stderr)
    probe.check_returncode()
    save(ROOT/'preflight.json',json.loads(probe.stdout))
    first,identity=phase(run,1,16,'STOP')
    initial=first['manifest']['num_timesteps']
    assert (run/'STOP').is_file()
    (run/'STOP').unlink()
    second,identity2=phase(run,2,initial+16,'SIGTERM')
    assert identity==identity2, 'Run identity changed'
    assert second['state']['sessions']==2
    assert second['manifest']['num_timesteps']>initial
    resumed=[r for r in second['reports'] if r['initial_steps']>=initial]
    assert resumed and resumed[0]['initial_steps']==initial
    assert all(r['learning_starts']==5000 and r['gradient_updates']==0 for r in second['reports'])
    assert second['reports'][-1]['final_replay_size']>=first['reports'][-1]['final_replay_size']+16
    old=Path(first['checkpoint'])
    for name in ('model.zip','replay.pkl'):assert digest(old/name)==first['manifest']['files'][name]
    result={'status':'passed','device':read(ROOT/'preflight.json')['cuda_probe']['gpu'],'profile':'formal','first_stopped_steps':initial,'resumed_initial_steps':resumed[0]['initial_steps'],'second_stopped_steps':second['manifest']['num_timesteps'],'first_replay_size':first['reports'][-1]['final_replay_size'],'second_replay_size':second['reports'][-1]['final_replay_size'],'same_run_identity':True,'checkpoint_hashes_verified':True,'original_checkpoint_unchanged':True,'stop_methods':['STOP','SIGTERM'],'owned_containers_after_each_exit':[0,0],'gradient_updates':0,'learning_starts':5000,'long_training_started':False,'navigation_quality_evaluated':False,'run_directory':str(run)}
    save(ROOT/'summary.json',result)
    print(json.dumps(result,indent=2),flush=True)

if __name__=='__main__':
    try:main()
    except Exception as exc:
        if ROOT.exists():save(ROOT/'acceptance-error.json',{'error':repr(exc)})
        raise
    finally:
        if ROOT.exists():export()
