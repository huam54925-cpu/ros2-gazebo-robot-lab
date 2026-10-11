#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"
python3 - <<'PY'
import json,subprocess
from pathlib import Path
for recorded in json.loads(Path('provenance/base-images.json').read_text()):
    tag=recorded['RepoTags'][0]
    actual=subprocess.check_output(['docker','image','inspect',tag,'--format','{{.Id}}'],text=True).strip()
    if actual != recorded['Id']:
        raise SystemExit(f'Base image changed: {tag}; review/update provenance before rebuilding')
PY
docker build -f docker/Dockerfile.sim-train -t robot-sim:local-drive-20261011 .
docker build -f docker/Dockerfile.trainer -t robot-trainer:local-drive-cpu-v2 .
