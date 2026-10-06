#!/usr/bin/env bash
# Explicit operator preparation: resume only after verified stop, then scan.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if (( $# > 1 )); then echo "Usage: initialize-exploration-map.sh [container-output-json]" >&2; exit 2; fi
OUTPUT=${1:-/work/robot_ws/log/map-initialization-$(date -u +%Y%m%dT%H%M%S)-$$.json}
"$ROOT/scripts/robot-skills.sh" resume
exec "$ROOT/docker/start-exploration.sh" --initialize-only --wall-budget 300 --output "$OUTPUT"
