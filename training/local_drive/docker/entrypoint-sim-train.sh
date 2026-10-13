#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/lyrical/setup.bash
set -u
if (( $# > 0 )); then exec "$@"; fi
WORLD=${ROBOT_WORLD:-/opt/robot-lab/workspace/navigation_base/worlds/indoor_mapping.sdf}
test -r "$WORLD"
exec gz sim -s --headless-rendering -v 3 "$WORLD"
