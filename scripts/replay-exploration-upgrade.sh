#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Offline host Python with numpy; no API key, ROS client, or motion command.
exec python3 "$ROOT/workspace/robot_skills/replay_upgrade.py" "$@"
