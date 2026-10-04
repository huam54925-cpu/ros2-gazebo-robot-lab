#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$(docker inspect --format '{{.State.Running}}' robot-sim-gui)" != true ]]; then
    echo 'Start the simulation and SLAM first; see docs/robot-readonly.md.' >&2
    exit 2
fi
mkdir -p "$ROOT/workspace/log"
if docker exec robot-sim-gui pgrep -f '^python3 /work/robot_ws/robot_agent/ros_status_bridge.py$' >/dev/null; then
    echo 'Read-only ROS bridge is already running.'
else
    docker exec -d robot-sim-gui bash -lc \
        'source /opt/ros/lyrical/setup.bash; exec python3 /work/robot_ws/robot_agent/ros_status_bridge.py >> /work/robot_ws/log/robot-status-bridge.log 2>&1'
fi
"$ROOT/.venv-agent/bin/python" - "$ROOT" <<'PY'
import json
from pathlib import Path
import sys
import time
snapshot = Path(sys.argv[1]) / 'workspace/log/robot-status.json'
for _ in range(30):
    try:
        data = json.loads(snapshot.read_text())
        if 0 <= time.monotonic() - data['generated_monotonic_s'] < 2:
            print('Read-only ROS bridge is live; MCP starts on demand via robot-status.sh.')
            break
    except (OSError, ValueError, KeyError):
        pass
    time.sleep(0.5)
else:
    raise SystemExit('Bridge is not producing fresh snapshots. Check workspace/log/robot-status-bridge.log.')
PY
