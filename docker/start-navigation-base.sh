#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if docker container inspect robot-sim-gui >/dev/null 2>&1; then
  echo 'robot-sim-gui already exists; stop the previous simulation first.' >&2
  exit 1
fi
: "${DISPLAY:?DISPLAY is required}"
test -r "$ROOT/gui/xauth"
exec docker run --rm -it --name robot-sim-gui --gpus all --network host \
  --user "$(id -u):$(id -g)" --shm-size=512m \
  -e DISPLAY -e XAUTHORITY=/tmp/robot.xauth -e QT_QPA_PLATFORM=xcb \
  -e HOME=/sim-home -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,display \
  -e LIBGL_ALWAYS_SOFTWARE=1 -e QSG_RENDER_LOOP=threaded \
  -e QT_XCB_GL_INTEGRATION=xcb_glx -e QSG_RHI_BACKEND=opengl \
  --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly \
  --mount "type=bind,src=$ROOT/gui/xauth,dst=/tmp/robot.xauth,readonly" \
  --mount "type=bind,src=$ROOT/home,dst=/sim-home" \
  --mount "type=bind,src=$ROOT/workspace,dst=/work/robot_ws" \
  robot-sim:lyrical-v1 bash -lc \
  'source /opt/ros/lyrical/setup.bash; exec ros2 launch /work/robot_ws/navigation_base/bringup.launch.py'
