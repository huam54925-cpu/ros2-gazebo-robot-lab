#!/usr/bin/env bash
# Fully offline build from already cached and inspected base images.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SIM_BASE=${SIM_BASE:-ghcr.io/huam54925-cpu/ros2-gazebo-robot-lab-sim:20261011}
TRAINER_BASE=${TRAINER_BASE:-robot-trainer:local-drive-cuda-v1}
docker image inspect "$SIM_BASE" "$TRAINER_BASE" >/dev/null
CONTEXT=$(mktemp -d)
trap 'rm -rf "$CONTEXT"' EXIT
cp -a "$ROOT"/. "$CONTEXT"/
cp -L "$(command -v docker)" "$CONTEXT/docker-cli"
python3 - "$ROOT" "$CONTEXT" "$SIM_BASE" "$TRAINER_BASE" <<'PY'
import hashlib,json,pathlib,subprocess,sys
root,context=map(pathlib.Path,sys.argv[1:3])
files={}
for directory in ('training','sim_sync','scripts','docker'):
 for p in sorted((root/directory).rglob('*')):
  if p.is_file() and '__pycache__' not in p.parts:files[str(p.relative_to(root))]=hashlib.sha256(p.read_bytes()).hexdigest()
manifest={'base_commit':subprocess.check_output(['git','-C',str(root),'rev-parse','HEAD'],text=True).strip(),'source_files':files,
 'base_images':{tag:subprocess.check_output(['docker','image','inspect',tag,'--format','{{.Id}}'],text=True).strip() for tag in sys.argv[3:]}}
(context/'source-manifest.json').write_text(json.dumps(manifest,sort_keys=True,indent=2))
PY
docker build --pull=false --network=none --build-arg BASE_IMAGE="$SIM_BASE" -f "$CONTEXT/docker/Dockerfile.sync-sim" -t local-drive-sync-sim:20261012-v2 "$CONTEXT"
docker build --pull=false --network=none --build-arg BASE_IMAGE="$TRAINER_BASE" -f "$CONTEXT/docker/Dockerfile.sync-trainer" -t local-drive-sync-trainer:20261012-v2 "$CONTEXT"
