"""Local simulation-only profile. The model cannot change this runtime file.

Activate only while stopped, then restart guard, navigation and status bridge.
The ordinary profile is unchanged when no activation file exists.
"""
import json
from pathlib import Path
ACTIVATION = Path(__file__).resolve().parents[1]/'log'/'safety-profile.json'
def active_profile():
    if not ACTIVATION.exists(): return 'radial_default'
    value=json.loads(ACTIVATION.read_text())
    if value.get('profile') not in ('radial_default','footprint_075'):
        raise ValueError('invalid_local_safety_profile')
    return value['profile']
PROFILE=active_profile()
FOOTPRINT_MODE=PROFILE=='footprint_075'
try:
    from .robot_contract import CONTRACT, CONTRACT_HASH
except ImportError:
    from robot_contract import CONTRACT, CONTRACT_HASH
BODY_PADDING_M=CONTRACT['body_padding_m']
PROFILE_LIMITS=CONTRACT['profiles'][PROFILE]
MAX_LINEAR_M_S=PROFILE_LIMITS['max_linear_m_s']
MAX_ANGULAR_RAD_S=PROFILE_LIMITS['max_angular_rad_s']
