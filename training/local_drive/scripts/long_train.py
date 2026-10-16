#!/usr/bin/env python3
"""Bounded LocalDrive SAC scheduling. Host stdlib only; dependencies stay in Docker."""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

PROJECT = Path(__file__).resolve().parent.parent
STOP_REQUESTED = False
LABEL = 'org.local-drive.run'

class StopRequested(Exception):
    pass

class OwnershipUncertain(RuntimeError):
    pass


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def atomic_json(path, obj):
    path = Path(path)
    temp = path.with_name('.'+path.name+'.'+uuid.uuid4().hex+'.tmp')
    with temp.open('w', encoding='utf-8') as f:
        json.dump(obj,f,indent=2,ensure_ascii=False,allow_nan=False)
        f.write('\n');f.flush();os.fsync(f.fileno())
    os.replace(temp,path)
    fd=os.open(path.parent,os.O_RDONLY)
    try: os.fsync(fd)
    finally: os.close(fd)


def event(out, **fields):
    with (out/'events.jsonl').open('a') as f:
        f.write(json.dumps({'utc':utc_now(),**fields})+'\n');f.flush();os.fsync(f.fileno())


def docker(args, timeout=30, check=True, **kw):
    """Bound the CLI, including daemon connection hangs, not just container stop."""
    return subprocess.run(['docker',*args],check=check,text=True,timeout=timeout,
                          stdout=kw.pop('stdout',subprocess.PIPE),
                          stderr=kw.pop('stderr',subprocess.PIPE),**kw)


def stop_handler(*_):
    global STOP_REQUESTED
    STOP_REQUESTED=True


def should_stop(out,deadline):
    return STOP_REQUESTED or (out/'STOP').exists() or time.monotonic()>=deadline


def verify_latest(root):
    root=Path(root);pointer=root/'latest.txt'
    if not pointer.exists():
        if list(root.glob('step-*')):raise RuntimeError('Checkpoint folders exist without latest.txt')
        return 0,None
    name=pointer.read_text().strip()
    if not name.startswith('step-') or Path(name).name!=name:raise RuntimeError('Invalid checkpoint pointer')
    leaf=root/name;m=json.loads((leaf/'manifest.json').read_text())
    if m.get('schema')!='local-drive-v2' or m.get('profile')!='formal' or m.get('environment')!='LocalDriveEnv':
        raise RuntimeError('Expected a formal LocalDrive-v2 checkpoint')
    for filename in ('model.zip','replay.pkl'):
        if sha256(leaf/filename)!=m['files'][filename]:raise RuntimeError('Checkpoint hash mismatch: '+filename)
    n=m['num_timesteps']
    if type(n)!=int or n<=0:raise RuntimeError('Invalid checkpoint steps')
    return n,leaf


def owned_containers(run_id):
    return docker(['ps','-aq','--filter',f'label={LABEL}={run_id}']).stdout.split()


def cleanup_owned(run_id):
    """Fail closed if destruction cannot be verified. Never touch unrelated runs."""
    try:
        # Remove learner/evaluator before simulators, then rescan for late children.
        for _ in range(4):
            ids=owned_containers(run_id)
            if not ids:return
            for cid in ids:
                info=json.loads(docker(['inspect',cid]).stdout)[0]
                if info['Config'].get('Labels',{}).get(LABEL)!=run_id:
                    raise RuntimeError('Container ownership changed')
            docker(['rm','-f',*ids],timeout=45,check=False)
        if owned_containers(run_id):raise RuntimeError('Owned containers remain')
    except Exception as exc:
        raise OwnershipUncertain('Cannot verify owned container cleanup; refusing another chunk') from exc


def common_args(out,run_id,name,role,socket,sim):
    return ['run','--detach','--init','--name',name,'--label',f'{LABEL}={run_id}',
            '--label',f'org.local-drive.role={role}','--pull','never','--network','none',
            '--gpus','all','--shm-size=1g','-v',f'{socket}:/var/run/docker.sock',
            '-v',f'{out}:/results','-e','LOCAL_DRIVE_SIM_GPU=0',
            '-e',f'LOCAL_DRIVE_SIM_IMAGE={sim}','-e',f'LOCAL_DRIVE_RUN_ID={run_id}']


def run_container(args, *, out,run_id,name,log_path,deadline,operation_seconds):
    stopped=False;started=time.monotonic()
    try:
        # Cleanup also runs if Docker accepted creation but the CLI timed out.
        docker(args,timeout=90)
        event(out,event='container_started',name=name)
        while True:
            info=json.loads(docker(['inspect',name]).stdout)[0]
            if not info['State']['Running']:
                return int(info['State']['ExitCode']),stopped
            if should_stop(out,deadline):
                stopped=True
                docker(['stop','--time','120',name],timeout=140,check=False)
                info=json.loads(docker(['inspect',name]).stdout)[0]
                if info['State']['Running']:raise RuntimeError('Container did not stop')
                return int(info['State']['ExitCode']),True
            if time.monotonic()-started>operation_seconds:
                docker(['stop','--time','120',name],timeout=140,check=False)
                raise RuntimeError('Operation watchdog expired')
            time.sleep(1)
    finally:
        try:
            with log_path.open('w') as f:
                docker(['logs',name],timeout=30,check=False,stdout=f,stderr=subprocess.STDOUT)
        finally:
            cleanup_owned(run_id)


def resolve_context():
    if os.environ.get('DOCKER_HOST'):
        raise RuntimeError('Unset DOCKER_HOST; select an explicit local Docker context')
    context=os.environ.get('DOCKER_CONTEXT') or docker(['context','show']).stdout.strip()
    os.environ['DOCKER_CONTEXT']=context
    host=docker(['context','inspect',context,'--format','{{(index .Endpoints "docker").Host}}']).stdout.strip()
    if not host.startswith('unix://') or not Path(host[7:]).exists():
        raise RuntimeError('A reachable local Unix Docker socket is required')
    docker(['info','--format','{{.ServerVersion}}'])
    return context,host[7:]


def inspect_image(tag):
    info=json.loads(docker(['image','inspect',tag]).stdout)[0]
    if info.get('Architecture')!='amd64':raise RuntimeError('Expected linux/amd64 image')
    return info['Id']


def check_image_sources(trainer):
    raw=docker(['run','--rm','--network','none','--pull','never','--entrypoint','cat',trainer,
                '/opt/robot-training/release-manifest.json'],timeout=60).stdout
    manifest=json.loads(raw)
    for rel,digest in manifest['source_hashes'].items():
        if sha256(PROJECT/rel)!=digest:raise RuntimeError('Host source differs from image: '+rel)
    return manifest


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-dir',required=True,type=Path)
    p.add_argument('--hours',type=float,default=3)
    p.add_argument('--chunk-steps',type=int,default=1000)
    p.add_argument('--eval-every',type=int,default=5000)
    p.add_argument('--eval-seed',type=int,default=7)
    p.add_argument('--seed',type=int,default=7)
    p.add_argument('--max-failures',type=int,default=3)
    p.add_argument('--chunk-timeout',type=float,default=1800)
    p.add_argument('--min-free-gb',type=float,default=10)
    a=p.parse_args(argv)
    if (not math.isfinite(a.hours) or not 0<a.hours<=168 or not 1<=a.chunk_steps<=1000000 or
        a.eval_every<0 or a.max_failures<0 or not 0<=a.seed<2**31 or not 0<=a.eval_seed<2**31 or
        not math.isfinite(a.chunk_timeout) or a.chunk_timeout<=0 or
        not math.isfinite(a.min_free_gb) or a.min_free_gb<0):p.error('Invalid budget or parameters')
    out=a.run_dir.expanduser().resolve();out.mkdir(parents=True,exist_ok=True)
    for name in ('logs','reports','evaluations'):(out/name).mkdir(exist_ok=True)
    with (out/'.lock').open('a') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise RuntimeError('Another manager owns this run directory')
        if (out/'STOP').exists():raise RuntimeError('Remove STOP explicitly before resuming')
        signal.signal(signal.SIGINT,stop_handler);signal.signal(signal.SIGTERM,stop_handler)
        context,socket=resolve_context()
        trainer=inspect_image(os.environ.get('TRAINER_IMAGE','local-drive-sync-trainer:20261016-longrun'))
        sim=inspect_image(os.environ.get('LOCAL_DRIVE_SIM_IMAGE','local-drive-sync-sim:20261012-v2'))
        manifest=check_image_sources(trainer)
        identity={'trainer_image_id':trainer,'sim_image_id':sim,'docker_context':context,
                  'source_hashes':manifest['source_hashes'],'seed':a.seed,'profile':'formal'}
        meta=out/'run-identity.json'
        if meta.exists():
            saved=json.loads(meta.read_text())
            if saved['identity']!=identity:raise RuntimeError('Identity changed; use a new directory')
            run_id=saved['run_id']
            if len(run_id)!=32 or any(c not in '0123456789abcdef' for c in run_id):raise RuntimeError('Invalid run identity')
        else:
            if (out/'checkpoints').exists():raise RuntimeError('Refusing checkpoints without run identity')
            run_id=uuid.uuid4().hex;atomic_json(meta,{'identity':identity,'run_id':run_id})
        cleanup_owned(run_id) # A previous killed host process may have left detached containers.
        state_path=out/'long-run-state.json'
        state=json.loads(state_path.read_text()) if state_path.exists() else {'chunks':0,'last_eval_step':0,'sessions':0}
        state['sessions']+=1
        deadline=time.monotonic()+a.hours*3600;failures=0
        def save(phase,**fields):
            state.update(phase=phase,updated_utc=utc_now(),**fields);atomic_json(state_path,state)
        try:
            save('starting')
            event(out,event='session_started',hours=a.hours,session=state['sessions'])
            while not should_stop(out,deadline):
                import shutil
                if shutil.disk_usage(out).free<a.min_free_gb*1024**3:
                    raise RuntimeError('Free disk space below configured threshold')
                before,_=verify_latest(out/'checkpoints')
                state['chunks']+=1
                number=state['chunks'];name=f'ldtrain-{run_id[:12]}-{number}'
                # Persist number and ownership BEFORE launch; crashes cannot reuse log names.
                save('training',valid_steps=before,active_container=name)
                args=common_args(out,run_id,name,'trainer',socket,sim)+[trainer,'--profile','formal',
                     '--device','cuda','--steps',str(a.chunk_steps),'--seed',str(a.seed),
                     '--resume-steps','0','--output','/results']
                if before:args+=['--resume','/results/checkpoints']
                report=out/'vehicle-training.json'
                if report.exists():report.unlink()
                requested=False
                try:
                    code,requested=run_container(args,out=out,run_id=run_id,name=name,
                        log_path=out/'logs'/f'chunk-{number:05d}.log',deadline=deadline,operation_seconds=a.chunk_timeout)
                    after,leaf=verify_latest(out/'checkpoints')
                    if report.exists():
                        (out/'reports'/f'chunk-{number:05d}.json').write_bytes(report.read_bytes())
                    event(out,event='chunk_finished',chunk=number,before=before,after=after,exit_code=code)
                    save('training',valid_steps=after,active_container=None)
                    if requested or should_stop(out,deadline):break
                    if code!=0 or after<=before:raise RuntimeError(f'Chunk failed: exit={code}, steps={before}->{after}')
                    failures=0
                except OwnershipUncertain:raise
                except Exception as exc:
                    failures+=1;event(out,event='chunk_error',chunk=number,error=repr(exc))
                    if should_stop(out,deadline):break
                    if failures>a.max_failures:raise
                    save('retrying',consecutive_failures=failures)
                    # No resubmission until run_container's ownership cleanup succeeded.
                    continue
                if a.eval_every and after-state['last_eval_step']>=a.eval_every and deadline-time.monotonic()>30:
                    name=f'ldeval-{run_id[:12]}-{number}'
                    rel=f'evaluations/step-{after}-seed-{a.eval_seed}-session-{state["sessions"]}-chunk-{number}'
                    save('evaluating',active_container=name)
                    # Immutable checkpoint leaf; no dependency on a moving latest pointer.
                    args=common_args(out,run_id,name,'evaluator',socket,sim)+[
                        '-v',f'{leaf}:/checkpoint:ro','--entrypoint','python',trainer,'-m','training.evaluate_vehicle',
                        '--checkpoint','/checkpoint','--output','/results/'+rel,'--device','cuda',
                        '--code-revision',manifest['base_revision'],'--seeds',str(a.eval_seed),'--max-steps','600']
                    code,requested=run_container(args,out=out,run_id=run_id,name=name,
                        log_path=out/'logs'/f'eval-{number:05d}.log',deadline=deadline,operation_seconds=600)
                    if requested or should_stop(out,deadline):break
                    if code:raise RuntimeError('Evaluation failed; inspect evaluation log')
                    state['last_eval_step']=after;save('training',active_container=None)
                    event(out,event='evaluation_completed',step=after,path=rel)
            steps,_=verify_latest(out/'checkpoints')
            save('stopped' if STOP_REQUESTED or (out/'STOP').exists() else 'budget_complete',valid_steps=steps,active_container=None)
            event(out,event='session_finished',phase=state['phase'],steps=steps)
        except BaseException as exc:
            try:save('failed',error=repr(exc))
            finally:cleanup_owned(run_id)
            raise
        print(json.dumps(state),flush=True)


if __name__=='__main__':
    try:main()
    except Exception as exc:
        print(f'FATAL: {exc}',file=sys.stderr);raise SystemExit(1)
