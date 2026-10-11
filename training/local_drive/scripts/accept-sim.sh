#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
OUT=${1:-"$ROOT/runs/sim-$(date -u +%Y%m%dT%H%M%SZ)"}
mkdir -p "$OUT"
IMAGE=${SIM_IMAGE:-robot-sim:local-drive-20261011}
NAME="local-drive-accept-$$"
cleanup() {
  docker logs "$NAME" > "$OUT/sim.log" 2>&1 || true
  docker rm -f "$NAME" >/dev/null 2>&1 || true
}
trap cleanup EXIT
# No source mounts, no display, no host network, no privileged mode.
docker run -d --name "$NAME" --gpus all --shm-size=1g \
  -e ROS_DOMAIN_ID=41 -e GZ_PARTITION="$NAME" "$IMAGE" > "$OUT/container-id.txt"
docker inspect "$NAME" > "$OUT/container-inspect.json"
timeout 150 docker exec "$NAME" /usr/local/bin/start-sim-train \
  python3 /opt/robot-lab/scripts/sim_probe.py > "$OUT/probe.json" 2> "$OUT/probe.stderr"
docker exec "$NAME" sh -c 'cat /root/.gz/rendering/ogre2.log' > "$OUT/ogre2.log"
grep -q 'EGL Subsystem' "$OUT/ogre2.log"
printf 'PASS: %s\n' "$OUT"
