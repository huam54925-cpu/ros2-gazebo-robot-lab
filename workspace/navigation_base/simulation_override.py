"""Explicit, expiring operator diagnostic override; off outside this Docker run."""
import json
import os
from pathlib import Path
import time

MARKER=Path(__file__).resolve().parents[1]/'log'/'gazebo-collision-bypass.json'


def active():
    if not Path('/.dockerenv').exists():return False
    try:
        record=json.loads(MARKER.read_text())
        return (record.get('mode')=='gazebo_collision_bypass' and record.get('enabled') is True
                and record.get('hostname')==os.uname().nodename
                and time.time()<record['expires_unix_s'])
    except (OSError,ValueError,KeyError,TypeError):return False


def unchecked():
    return {'safe':True,'reason':'operator_gazebo_collision_checks_suspended',
            'collision_checks':False,'safety_verified':False,'scope':'temporary_simulation_diagnostic'}
