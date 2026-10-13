#!/usr/bin/env bash
# Copy beside the exported bundle files as run-offline.sh.
set -euo pipefail
BUNDLE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$BUNDLE"
printf 'Checking bundle checksums...\n'
sha256sum -c SHA256SUMS
command -v docker >/dev/null
export DOCKER_CONTEXT=${DOCKER_CONTEXT:-wsl-native}
ENDPOINT=$(docker context inspect "$DOCKER_CONTEXT" --format '{{(index .Endpoints "docker").Host}}')
case "$ENDPOINT" in unix://*) SOCKET=${ENDPOINT#unix://} ;; *) echo "A local unix Docker socket is required" >&2; exit 1 ;; esac
docker info >/dev/null
printf 'Importing offline images...\n'
docker load -i local-drive-images.tar.gz
SIM_IMAGE=local-drive-sync-sim:20261012-v2
TRAINER_IMAGE=local-drive-sync-trainer:20261012-v2
docker image inspect "$SIM_IMAGE" "$TRAINER_IMAGE" >/dev/null
RESULTS=${1:-"$BUNDLE/results-$(date -u +%Y%m%dT%H%M%SZ)"}
if [[ -e "$RESULTS" ]]; then echo "Output directory already exists: $RESULTS" >&2; exit 1; fi
mkdir -p "$RESULTS"
RESULTS=$(cd "$RESULTS" && pwd)
COMMON=(--rm --pull never --network none --gpus all --shm-size=1g -v "$SOCKET:/var/run/docker.sock" -e LOCAL_DRIVE_SIM_GPU=0 -v "$RESULTS:/results")
printf 'Running unit and transport tests...\n'
docker run "${COMMON[@]}" --entrypoint python "$TRAINER_IMAGE" -m unittest discover -s tests -v > "$RESULTS/unit-tests.log" 2>&1
docker run --rm --pull never --network none "$SIM_IMAGE" python3 /src/training/local_drive/sim_sync/test_service.py > "$RESULTS/service-tests.log" 2>&1
# Each gate must pass before the next command runs.
printf 'Running real synchronous acceptance (logs: %s/acceptance.log)...\n' "$RESULTS"
docker run "${COMMON[@]}" --entrypoint python "$TRAINER_IMAGE" scripts/accept_sync.py --stage all --output /results/acceptance > "$RESULTS/acceptance.log" 2>&1
printf 'Running CUDA vehicle training and recovery (logs: %s/training.log)...\n' "$RESULTS"
docker run "${COMMON[@]}" "$TRAINER_IMAGE" --profile acceptance --device cuda --steps 64 --resume-steps 8 --output /results/training > "$RESULTS/training.log" 2>&1
printf 'PASS. Results: %s\n' "$RESULTS"
