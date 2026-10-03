#!/usr/bin/env bash
set -euo pipefail
exec docker exec -i robot-nav2 bash -lc 'source /opt/ros/lyrical/setup.bash; exec python3 /work/robot_ws/navigation_base/exploration/explore.py "$@"' bash "$@"
