#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -x "$ROOT/.venv-agent/bin/python" ]]; then
    echo 'Run scripts/setup-agent-env.sh first.' >&2
    exit 2
fi
exec "$ROOT/.venv-agent/bin/python" "$ROOT/workspace/robot_agent/check_environment.py" "$@"
