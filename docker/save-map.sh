#!/usr/bin/env bash
set -euo pipefail
if (( $# > 1 )); then echo 'Usage: save-map.sh [map-name]' >&2; exit 2; fi
NAME=${1:-indoor-$(date +%Y%m%d-%H%M%S)}
if [[ ! "$NAME" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]]; then
  echo 'Use letters, numbers, underscores or hyphens for the map name.' >&2
  exit 2
fi
exec docker exec -i robot-sim-gui bash -lc '
set -e
source /opt/ros/lyrical/setup.bash
mkdir -p /work/robot_ws/maps
prefix=/work/robot_ws/maps/$1
if [[ -e "$prefix.yaml" || -e "$prefix.pgm" ]]; then
  echo "Map already exists: $prefix" >&2
  exit 1
fi
exec ros2 run nav2_map_server map_saver_cli -f "$prefix" \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=20.0
' bash "$NAME"
