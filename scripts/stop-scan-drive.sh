#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python3 - "$ROOT" <<'PY'
from pathlib import Path
import os,signal,sys
root=Path(sys.argv[1]);count=0
for d in (root/'workspace/log').glob('scan-drive-*'):
    if not d.is_dir():continue
    (d/'STOP').touch()
    f=d/'runner.pid'
    if not f.exists():continue
    try:
        pid=int(f.read_text());cmd=Path(f'/proc/{pid}/cmdline').read_bytes()
        if str(root/'workspace/robot_agent/run_scan_drive.py').encode() in cmd:
            os.kill(pid,signal.SIGTERM);count+=1
    except (FileNotFoundError,ProcessLookupError):pass
print(f'Stop requested for {count} active v2 runner(s). Inspect final stopped feedback.')
PY
