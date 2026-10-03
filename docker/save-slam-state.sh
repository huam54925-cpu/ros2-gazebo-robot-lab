#!/usr/bin/env bash
set -euo pipefail
if (( $# > 1 )); then echo 'Usage: save-slam-state.sh [name]' >&2; exit 2; fi
NAME=${1:-indoor-state-$(date +%Y%m%d-%H%M%S)}
if [[ ! "$NAME" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]]; then
  echo 'Use letters, numbers, underscores or hyphens for the name.' >&2; exit 2
fi
docker exec -i robot-sim-gui bash -lc 'source /opt/ros/lyrical/setup.bash; python3 - "$1"' bash "$NAME" <<'PY'
import sys
from pathlib import Path
import rclpy
from slam_toolbox.srv import SerializePoseGraph
prefix=Path('/work/robot_ws/maps')/sys.argv[1]
if any(prefix.with_suffix(s).exists() for s in ['.posegraph','.data']):
    raise SystemExit('Refusing to overwrite existing SLAM state')
prefix.parent.mkdir(parents=True,exist_ok=True)
rclpy.init()
node=rclpy.create_node('save_slam_state')
try:
    client=node.create_client(SerializePoseGraph,'/slam_toolbox/serialize_map')
    if not client.wait_for_service(timeout_sec=10):raise RuntimeError('SLAM serialization service unavailable')
    request=SerializePoseGraph.Request();request.filename=str(prefix)
    future=client.call_async(request)
    rclpy.spin_until_future_complete(node,future,timeout_sec=60)
    if not future.done():raise RuntimeError('Serialization timed out; inspect files before retrying')
    response=future.result()
    if response is None or response.result!=0:raise RuntimeError('SLAM failed to write state')
    for suffix in ['.posegraph','.data']:
        path=prefix.with_suffix(suffix)
        if not path.exists() or path.stat().st_size==0:raise RuntimeError('Missing or empty '+str(path))
        print(str(path),path.stat().st_size,'bytes')
finally:
    node.destroy_node();rclpy.shutdown()
PY
