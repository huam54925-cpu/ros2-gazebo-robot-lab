#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
# Reuse Docker's existing GHCR login. Never put a token in this script.
REGISTRY=ghcr.io/huam54925-cpu
for KIND in sim trainer; do
  if [[ "$KIND" == sim ]]; then
    LOCAL=robot-sim:local-drive-20261011
    VERSION=20261011
    DESCRIPTION='ROS 2 Lyrical Gazebo 10.5 headless EGL simulator; fixed scene smoke accepted'
  else
    LOCAL=robot-trainer:local-drive-cpu-v2
    VERSION=20261011-cpu-v2
    DESCRIPTION='CPU SAC toolchain smoke environment; Gazebo training backend pending'
  fi
  TARGET="$REGISTRY/ros2-gazebo-robot-lab-$KIND:$VERSION"
  docker image inspect "$LOCAL" >/dev/null
  docker build -f docker/Dockerfile.publish --build-arg "IMAGE=$LOCAL" \
    --build-arg "VERSION=$VERSION" --build-arg "DESCRIPTION=$DESCRIPTION" -t "$TARGET" .
  docker push "$TARGET"
  docker image inspect "$TARGET" --format '{{json .RepoDigests}}'
done
