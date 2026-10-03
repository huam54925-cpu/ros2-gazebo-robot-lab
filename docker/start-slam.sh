#!/usr/bin/env bash
set -euo pipefail
if (( $# != 0 )); then echo 'Usage: start-slam.sh' >&2; exit 2; fi
EXEC_ARGS=(-i)
if [[ -t 0 && -t 1 ]]; then EXEC_ARGS+=(-t); fi
exec docker exec "${EXEC_ARGS[@]}" robot-sim-gui bash -lc '
source /opt/ros/lyrical/setup.bash
ros2 pkg prefix slam_toolbox >/dev/null || {
  echo "Start the mapping image first: ./docker/start-navigation-base.sh mapping" >&2
  exit 1
}
if pgrep -f "^/opt/ros/lyrical/lib/slam_toolbox/async_slam_toolbox_node" >/dev/null; then
  echo "SLAM Toolbox is already running." >&2
  exit 1
fi
exec ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  slam_params_file:=/work/robot_ws/navigation_base/slam/mapper_params_online_async.yaml
'
