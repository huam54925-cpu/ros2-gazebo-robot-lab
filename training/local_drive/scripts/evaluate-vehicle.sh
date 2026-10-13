#!/usr/bin/env bash
set -euo pipefail
if (( $# < 2 )); then
  echo 'Usage: evaluate-vehicle.sh CHECKPOINT_ROOT NEW_OUTPUT_DIR [evaluator options]' >&2
  exit 2
fi
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
CHECKPOINT=$(realpath -- "$1")
OUTPUT=$(realpath -m -- "$2")
shift 2
if [[ -e "$OUTPUT" ]]; then echo "Output already exists: $OUTPUT" >&2; exit 1; fi
export DOCKER_CONTEXT=${DOCKER_CONTEXT:-wsl-native}
TRAINER_IMAGE=${TRAINER_IMAGE:-local-drive-sync-trainer:20261012-v2}
ENDPOINT=$(docker context inspect "$DOCKER_CONTEXT" --format '{{(index .Endpoints "docker").Host}}')
case "$ENDPOINT" in unix://*) SOCKET=${ENDPOINT#unix://} ;; *) echo 'Local Docker socket required' >&2; exit 1 ;; esac
REVISION=$(git -C "$ROOT" rev-parse HEAD)
mkdir -p "$OUTPUT"
# Freeze the new evaluator; the environment/backend remain those packaged in the image.
cp "$ROOT/training/evaluate_vehicle.py" "$OUTPUT/evaluate_vehicle.py"
docker image inspect "$TRAINER_IMAGE" > "$OUTPUT/trainer-image.json"
NAME="local-drive-eval-$(date -u +%Y%m%dT%H%M%SZ)-$$"
trap 'docker stop --time 90 "$NAME" >/dev/null 2>&1 || true; exit 130' INT TERM
docker run --rm --init --name "$NAME" --pull never --network none --gpus all --shm-size=1g \
  -v "$SOCKET:/var/run/docker.sock" -e LOCAL_DRIVE_SIM_GPU=0 \
  -v "$CHECKPOINT:/checkpoint:ro" -v "$OUTPUT:/results" \
  -v "$OUTPUT/evaluate_vehicle.py:/opt/robot-training/training/evaluate_vehicle.py:ro" \
  --entrypoint python "$TRAINER_IMAGE" -m training.evaluate_vehicle \
  --checkpoint /checkpoint --output /results/evaluation --device cuda \
  --code-revision "$REVISION" "$@" 2>&1 | tee "$OUTPUT/evaluation.log"
