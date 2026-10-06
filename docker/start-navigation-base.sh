#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SCENE=${1:-lidar}
case "$SCENE" in
  lidar) WORLD=/work/robot_ws/navigation_base/vehicle.world.sdf; DEFAULT_IMAGE=robot-sim:lyrical-v1; RVIZ=vehicle.rviz ;;
  mapping) WORLD=/work/robot_ws/navigation_base/worlds/indoor_mapping.sdf; DEFAULT_IMAGE=robot-sim:lyrical-mapping-v2; RVIZ=slam/mapping.rviz ;;
  *) echo 'Usage: start-navigation-base.sh [lidar|mapping]' >&2; exit 2 ;;
esac
WORLD=${ROBOT_WORLD:-$WORLD}
RVIZ=${ROBOT_RVIZ_CONFIG:-$RVIZ}
if (( $# > 1 )); then echo 'Expected at most one scene argument.' >&2; exit 2; fi
IMAGE=${ROBOT_IMAGE:-$DEFAULT_IMAGE}
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "Image missing: $IMAGE. See docs/indoor-mapping.md for build instructions." >&2
  exit 1
fi
if docker container inspect robot-sim-gui >/dev/null 2>&1; then
  echo 'robot-sim-gui already exists; stop the previous simulation first.' >&2
  exit 1
fi
: "${DISPLAY:?DISPLAY is required}"
test -r "$ROOT/gui/xauth"
# Pass through device groups for non-root EGL/GPU lidar access.
DEVICE_GROUPS=()
for device in /dev/dri/card* /dev/dri/renderD*; do
  [[ -e "$device" ]] && DEVICE_GROUPS+=(--group-add "$(stat -c %g "$device")")
done
TTY=()
[[ -t 0 && -t 1 ]] && TTY=(-it)
exec docker run --rm "${TTY[@]}" "${DEVICE_GROUPS[@]}" --name robot-sim-gui --gpus all --device /dev/dri:/dev/dri --network host \
  --user "$(id -u):$(id -g)" --shm-size=512m \
  -e DISPLAY -e XAUTHORITY=/tmp/robot.xauth -e QT_QPA_PLATFORM=xcb \
  -e HOME=/sim-home -e NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility,display \
  -e QSG_RENDER_LOOP=threaded \
  -e ROBOT_DIRECT_GAZEBO="${ROBOT_DIRECT_GAZEBO:-false}" \
  -e QT_XCB_GL_INTEGRATION=xcb_glx -e QSG_RHI_BACKEND=opengl \
  --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly \
  --mount "type=bind,src=$ROOT/gui/xauth,dst=/tmp/robot.xauth,readonly" \
  --mount "type=bind,src=$ROOT/home,dst=/sim-home" \
  --mount "type=bind,src=$ROOT/workspace,dst=/work/robot_ws" \
  "$IMAGE" bash -lc \
  'source /opt/ros/lyrical/setup.bash; exec ros2 launch /work/robot_ws/navigation_base/bringup.launch.py "world:=$1" "rviz_config:=/work/robot_ws/navigation_base/$2" "headless:=$3" "direct_gazebo:=$ROBOT_DIRECT_GAZEBO"' bash "$WORLD" "$RVIZ" "${ROBOT_HEADLESS:-false}"
