#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
: "${BUILD_DIR:?Set BUILD_DIR to a new directory on a data disk}"
if [[ -e "$BUILD_DIR" ]]; then echo 'BUILD_DIR must be new' >&2; exit 1; fi
mkdir -p "$BUILD_DIR"
cp -a "$ROOT/training" "$ROOT/scripts" "$ROOT/tests" "$BUILD_DIR/"
find "$BUILD_DIR" -type d -name __pycache__ -prune -exec rm -rf {} +
cp "$ROOT/docker/Dockerfile.longrun" "$BUILD_DIR/Dockerfile"
python3 - "$ROOT" "$BUILD_DIR" <<'PY'
import hashlib,json,subprocess,sys
from pathlib import Path
src,out=map(Path,sys.argv[1:]);files={}
for folder in ('training','scripts'):
 for p in sorted((out/folder).rglob('*')):
  if p.is_file():files[str(p.relative_to(out))]=hashlib.sha256(p.read_bytes()).hexdigest()
manifest={'base_revision':subprocess.check_output(['git','-C',str(src),'rev-parse','HEAD'],text=True).strip(),
          'source_hashes':files,'dependency_changes':False,'platform':'linux/amd64'}
(out/'release-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
PY
docker build --network none --pull=false --build-arg BASE_IMAGE="${BASE_IMAGE:-local-drive-sync-trainer:20261012-v2}" \
  -t "${TRAINER_IMAGE:-local-drive-sync-trainer:20261016-longrun}" "$BUILD_DIR"
