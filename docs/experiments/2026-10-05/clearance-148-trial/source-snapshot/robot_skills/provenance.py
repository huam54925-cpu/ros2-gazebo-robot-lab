"""Hash source/configuration inputs without reading credentials or runtime logs."""
import hashlib
from pathlib import Path


def source_manifest():
    workspace=Path(__file__).resolve().parents[1]
    files=[]
    for directory in ('robot_agent','robot_skills','navigation_base/exploration','exploration_support'):
        files.extend((workspace/directory).glob('*.py'))
    files.extend(workspace/path for path in ('navigation_base/navigation/nav2.yaml',
        'navigation_base/safety_contract.py','navigation_base/map_identity.py','robot_agent/motion_trial.xml'))
    return {str(path.relative_to(workspace)):hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(files) if not path.name.startswith('test_')}
