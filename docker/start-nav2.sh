#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
IMAGE=${ROBOT_NAV_IMAGE:-robot-sim:lyrical-navigation-v1}
if [[ "$(docker inspect --format '{{.State.Running}}' robot-sim-gui)" != true ]]; then
  echo 'Start the indoor simulation and SLAM first.' >&2; exit 1
fi
if docker inspect robot-nav2 >/dev/null 2>&1; then
  echo 'robot-nav2 already exists; stop it before starting another navigation stack.' >&2; exit 1
fi
exec docker run --rm --init --name robot-nav2 --network host \
  --user "$(id -u):$(id -g)" -e HOME=/sim-home \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  --mount "type=bind,src=$ROOT/home,dst=/sim-home" \
  --mount "type=bind,src=$ROOT/workspace,dst=/work/robot_ws" \
  "$IMAGE" bash -lc 'source /opt/ros/lyrical/setup.bash; python3 /work/robot_ws/navigation_base/slam/check_slam.py && exec ros2 launch /work/robot_ws/navigation_base/navigation/navigation.launch.py'
