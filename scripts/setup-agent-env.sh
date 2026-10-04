#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export UV_CACHE_DIR="$ROOT/.tools/uv-cache"
export UV_NO_MANAGED_PYTHON=1
python3 "$ROOT/scripts/bootstrap-agent-uv.py"
if [[ ! -x "$ROOT/.venv-agent/bin/python" ]]; then
    "$ROOT/.tools/uv" venv --python python3 "$ROOT/.venv-agent"
fi
"$ROOT/.tools/uv" pip sync --python "$ROOT/.venv-agent/bin/python" \
    --require-hashes "$ROOT/workspace/robot_agent/requirements.lock"
"$ROOT/.venv-agent/bin/python" - "$ROOT" <<'PY'
import os
from pathlib import Path
import sys
root = Path(sys.argv[1])
try:
    fd = os.open(root / '.env', os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
except FileExistsError:
    print('Existing .env preserved')
else:
    with os.fdopen(fd, 'w') as f:
        f.write((root / '.env.example').read_text())
    print('Created private .env template (key not configured)')
PY
"$ROOT/scripts/check-agent-env.sh"
