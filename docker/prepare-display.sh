#!/usr/bin/env bash
# Refresh the local X11 cookie without broadening the X server access list.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
: "${DISPLAY:?Run this script in a graphical desktop terminal}"
command -v xauth >/dev/null || { echo 'Install xauth on the host first.' >&2; exit 1; }
mkdir -p "$ROOT/gui" "$ROOT/home/.rviz2" "$ROOT/logs" "$ROOT/workspace"
umask 077
ROBOT_AUTH_TMP=$(mktemp "$ROOT/gui/xauth.XXXXXX")
trap 'rm -f -- "$ROBOT_AUTH_TMP"' EXIT
ROBOT_COOKIE=$(xauth nlist "$DISPLAY")
if [[ -z "$ROBOT_COOKIE" ]]; then
  echo 'No X11 cookie found. Check DISPLAY and XAUTHORITY in your desktop session.' >&2
  exit 1
fi
printf '%s\n' "$ROBOT_COOKIE" | sed 's/^..../ffff/' | xauth -f "$ROBOT_AUTH_TMP" nmerge -
# Keep the existing inode so an existing bind mount remains valid.
cat "$ROBOT_AUTH_TMP" > "$ROOT/gui/xauth"
chmod 600 "$ROOT/gui/xauth"
echo 'Display authorization prepared in gui/xauth (not tracked by Git).'
