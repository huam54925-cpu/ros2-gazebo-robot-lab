#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
# Reuse Docker's existing GHCR login. Never put a token in this script.
REGISTRY=ghcr.io/huam54925-cpu
KINDS=("$@")
if (( ${#KINDS[@]} == 0 )); then KINDS=(sim trainer); fi
for KIND in "${KINDS[@]}"; do
  SUFFIX=trainer
  if [[ "$KIND" == sim ]]; then
    SUFFIX=sim
    LOCAL=robot-sim:local-drive-20261011
    VERSION=20261011
    DESCRIPTION='ROS 2 Lyrical Gazebo 10.5 headless EGL simulator; fixed scene smoke accepted'
  elif [[ "$KIND" == trainer ]]; then
    LOCAL=robot-trainer:local-drive-cpu-v2
    VERSION=20261011-cpu-v2
    DESCRIPTION='CPU SAC toolchain smoke environment; Gazebo training backend pending'
  elif [[ "$KIND" == cuda ]]; then
    LOCAL=robot-trainer:local-drive-cuda-v1
    VERSION=20261011-cu126-v1
    DESCRIPTION='CUDA 12.6 SAC toolchain smoke environment; GTX 1650 accepted; Gazebo training backend pending'
  else
    echo "Unknown image kind: $KIND (use sim, trainer, cuda)" >&2
    exit 2
  fi
  TARGET="$REGISTRY/ros2-gazebo-robot-lab-$SUFFIX:$VERSION"
  docker image inspect "$LOCAL" >/dev/null
  docker build -f docker/Dockerfile.publish --build-arg "IMAGE=$LOCAL" \
    --build-arg "VERSION=$VERSION" --build-arg "DESCRIPTION=$DESCRIPTION" -t "$TARGET" .
  docker push "$TARGET"
  docker image inspect "$TARGET" --format '{{json .RepoDigests}}'
done
