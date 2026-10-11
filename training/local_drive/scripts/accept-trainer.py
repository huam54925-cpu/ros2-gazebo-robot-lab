"""Normal save/reload/resume and graceful SIGTERM acceptance, bounded on host."""
import argparse
import json
import hashlib
import os
from pathlib import Path
import subprocess
import time
import uuid

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('output',type=Path)
p.add_argument('--device',choices=['cpu','cuda','auto'],default='cpu')
p.add_argument('--gpus',action='store_true',help='Expose NVIDIA GPUs to the container')
a=p.parse_args()
if a.device=='cuda' and not a.gpus: p.error('--device cuda requires --gpus')
gpu_args=['--gpus','all'] if a.gpus else []
out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
image=os.environ.get('TRAINER_IMAGE','robot-trainer:local-drive-cpu-v2')
uid=f'{os.getuid()}:{os.getgid()}'
def run(args,**kw):return subprocess.run(args,check=True,text=True,**kw)
with (out/'unit-tests.log').open('w') as log:
    run(['docker','run','--rm',*gpu_args,'--entrypoint','python',image,'-m','unittest','discover','-s','tests','-v'],stdout=log,stderr=subprocess.STDOUT,timeout=90)
normal=out/'normal';normal.mkdir()
with (out/'normal.log').open('w') as log:
    run(['docker','run','--rm',*gpu_args,'--user',uid,'-v',f'{normal}:/runs',image,'--device',a.device,'--steps','128'],stdout=log,stderr=subprocess.STDOUT,timeout=180)
report=json.loads((normal/'trainer-smoke.json').read_text());assert report['status']=='passed'
assert report['requested_device']==a.device
if a.device!='auto': assert report['device']==a.device
stop=out/'interrupted';stop.mkdir();name='local-drive-stop-'+uuid.uuid4().hex[:8]
try:
    run(['docker','run','-d',*gpu_args,'--name',name,'--user',uid,'-v',f'{stop}:/runs',image,'--device',a.device,'--steps','10000'],capture_output=True,timeout=60)
    deadline=time.monotonic()+60
    while not (stop/'ready.json').exists():
        if time.monotonic()>deadline:raise TimeoutError('Trainer did not become ready')
        time.sleep(.1)
    time.sleep(.25)
    run(['docker','kill','--signal','TERM',name],capture_output=True,timeout=10)
    rc=run(['docker','wait',name],capture_output=True,timeout=60).stdout.strip()
    assert rc=='0',rc
    interrupted=json.loads((stop/'trainer-smoke.json').read_text())
    assert interrupted['status']=='stopped',interrupted
    assert interrupted['device']==report['device']
    for folder in (normal,stop):
        cp=folder/'checkpoints'
        latest=cp/(cp/'latest.txt').read_text().strip()
        manifest=json.loads((latest/'manifest.json').read_text())
        for file,expected in manifest['files'].items():
            assert hashlib.sha256((latest/file).read_bytes()).hexdigest()==expected
    # train.py also compared deterministic predictions and restored replay sizes.
    (out/'acceptance.json').write_text(json.dumps({'status':'passed','normal':report,'sigterm':interrupted},indent=2)+'\n')
finally:
    with (out/'interrupted.log').open('w') as log:
        subprocess.run(['docker','logs',name],stdout=log,stderr=subprocess.STDOUT,timeout=10)
    subprocess.run(['docker','rm','-f',name],capture_output=True,timeout=20)
print('PASS:',out)
