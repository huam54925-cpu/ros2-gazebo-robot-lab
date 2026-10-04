#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$ROOT/.venv-agent/bin/python" "$ROOT/workspace/robot_agent/run_frontier.py" "$@"
