"""Publish complete checkpoints atomically on one filesystem; verify before load."""
import hashlib
import json
import os
from pathlib import Path
from uuid import uuid4

def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024), b''): h.update(block)
    return h.hexdigest()

def save_checkpoint(model, root, metadata):
    root=Path(root); root.mkdir(parents=True,exist_ok=True)
    name=f'step-{model.num_timesteps}-{uuid4().hex[:8]}'
    staging=root/f'.{name}.tmp'; staging.mkdir()
    model.save(str(staging/'model.zip'))
    model.save_replay_buffer(str(staging/'replay.pkl'))
    manifest=dict(metadata, num_timesteps=int(model.num_timesteps),
                  files={p:digest(staging/p) for p in ('model.zip','replay.pkl')})
    (staging/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    os.replace(staging,root/name)
    pointer=root/f'.latest-{uuid4().hex}.tmp'; pointer.write_text(name+'\n')
    os.replace(pointer,root/'latest.txt')
    return root/name

def verify_checkpoint(root):
    root=Path(root); name=(root/'latest.txt').read_text().strip()
    if not name.startswith('step-') or Path(name).name != name:
        raise ValueError('Invalid checkpoint pointer')
    path=root/name; manifest=json.loads((path/'manifest.json').read_text())
    for p in ('model.zip','replay.pkl'):
        if digest(path/p) != manifest['files'][p]: raise ValueError(f'Corrupt checkpoint: {p}')
    return path,manifest
