import hashlib
import json
import shutil
import subprocess
from pathlib import Path
from datetime import datetime, timezone

src = Path('/home/user111/AI-Training/runs/sac-longrun-20261017')
dst = Path('/mnt/f/AI-Training/diagnostics/sac-longrun-20261017-archive')
dst.mkdir(parents=True, exist_ok=True)
def sha(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda: f.read(4*1024*1024), b''): h.update(b)
    return h.hexdigest()
def copy(p, rel):
    q = dst / rel
    q.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(p, q)
    assert sha(p) == sha(q), str(p)
def write(name, obj):
    (dst/name).write_text(json.dumps(obj, ensure_ascii=False, indent=2)+'\n')
state = json.loads((src/'long-run-state.json').read_text())
assert state['phase'] == 'budget_complete' and state['valid_steps'] == 26144
assert not subprocess.check_output(['docker','--context','wsl-native','ps','-q'], text=True).strip()
for name in ['events.jsonl','long-run-state.json','run-identity.json','resource-limits.json','vehicle-training.json','manager-console.log','launch-preflight.stdout.txt','launch-preflight.stderr.txt']:
    copy(src/name, Path('runtime')/name)
for folder in ['reports','logs','evaluations','research']:
    for p in sorted((src/folder).rglob('*')):
        if p.is_file():
            assert p.stat().st_size < 10*1024**2
            copy(p,p.relative_to(src))
for p in Path('/home/user111/AI-Training/launchers/memory-bounded-20261017').iterdir():
    if p.is_file(): copy(p,Path('host-launcher')/p.name)
index={}
eval_summary=[]
for step in [5000,10000,15000,20000,25000]:
    reports=list((src/'evaluations').glob(f'step-{step}-*/evaluation.json'))
    assert len(reports)==1
    report=json.loads(reports[0].read_text())
    expected=report['checkpoint_hashes_before']
    assert expected==report['checkpoint_hashes_after']
    matches=[p for p in (src/'checkpoints').glob(f'step-{step}-*') if all(sha(p/n)==expected[n] for n in ['model.zip','manifest.json'])]
    assert len(matches)==1, (step,matches)
    ck=matches[0]
    assert sha(ck/'replay.pkl')==expected['replay.pkl']
    for n in ['model.zip','manifest.json']: copy(ck/n,Path('models')/ck.name/n)
    index[str(step)]={'directory':ck.name,'hashes':expected,'three_file_hashes_verified':True}
    eval_summary.append({'step':step,'episodes':report['episodes'],'metrics':report['metrics']})
latest=(src/'checkpoints/latest.txt').read_text().strip()
ck=src/'checkpoints'/latest
manifest=json.loads((ck/'manifest.json').read_text())
assert manifest['num_timesteps']==26144
for n,h in manifest['files'].items(): assert sha(ck/n)==h
for n in ['model.zip','manifest.json']: copy(ck/n,Path('models')/'final-26144'/n)
copy(src/'checkpoints/latest.txt',Path('runtime/latest.txt'))
index['final']={'directory':latest,'hashes':{n:sha(ck/n) for n in ['model.zip','manifest.json','replay.pkl']},'model_and_replay_verified':True}
inventory=[]
for p in sorted((src/'checkpoints').glob('*/manifest.json')):
    m=json.loads(p.read_text())
    copy(p,Path('checkpoint-manifests')/p.parent.name/p.name)
    inventory.append({'directory':p.parent.name,'steps':m['num_timesteps'],'files':[{'name':n,'bytes':(p.parent/n).stat().st_size,'manifest_sha256':h} for n,h in m['files'].items()]})
write('model-index.json',index)
write('evaluation-summary.json',eval_summary)
write('local-checkpoint-inventory.json',{'local_run':str(src),'checkpoint_count':len(inventory),'total_bytes':sum(p.stat().st_size for p in (src/'checkpoints').rglob('*') if p.is_file()),'verification_scope':'Six selected checkpoints verified; other hashes recorded from original manifests, sizes/existence checked. All original files retained locally.','checkpoints':inventory})
write('archive-verification.json',{'checked_at_utc':datetime.now(timezone.utc).isoformat(),'phase':state['phase'],'valid_steps':state['valid_steps'],'final_replay_size':json.loads((src/'vehicle-training.json').read_text())['final_replay_size'],'selected_checkpoint_count':6,'all_selected_hashes_verified':True,'active_containers':0,'large_files_uploaded':False})
print(json.dumps({'archive':str(dst),'files':sum(p.is_file() for p in dst.rglob('*')),'bytes':sum(p.stat().st_size for p in dst.rglob('*') if p.is_file()),'evaluations':[{'step':e['step'],'outcome':e['episodes'][0]['outcome'],'sim_seconds':e['episodes'][0]['sim_elapsed_s']} for e in eval_summary]},indent=2),flush=True)
subprocess.run(['sync'],check=True)
