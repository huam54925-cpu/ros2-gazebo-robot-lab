#!/usr/bin/env bash
# Distributed at the root of the offline bundle. Performs no package installation.
set -euo pipefail
BUNDLE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
cd "$BUNDLE"
command -v python3 >/dev/null
command -v docker >/dev/null
: "${DOCKER_CONTEXT:=wsl-native}"
export DOCKER_CONTEXT
sha256sum -c SHA256SUMS
python3 - <<'PY'
import json,subprocess
images=json.load(open('images.json'))
missing=False
for tag,expected in images.items():
 p=subprocess.run(['docker','image','inspect',tag,'--format','{{.Id}}'],capture_output=True,text=True,timeout=30)
 if p.returncode or p.stdout.strip()!=expected:missing=True
if missing:
 subprocess.run(['docker','load','-i','local-drive-images.tar.gz'],check=True,timeout=1800)
for tag,expected in images.items():
 got=subprocess.check_output(['docker','image','inspect',tag,'--format','{{.Id}}'],text=True,timeout=30).strip()
 if got!=expected:raise SystemExit('Image ID mismatch: '+tag)
print('Both pinned images available; no host packages installed.')
PY
if [[ -e source ]]; then
  echo 'source already exists; preserve it. Use a fresh bundle directory for a fresh extraction.'
else
  mkdir source
  tar -xzf local-drive-source.tar.gz -C source
fi
printf '\nSource: %s/source/training/local_drive\n' "$BUNDLE"
printf 'Next: follow 使用说明.txt, verify Windows/WSL data-disk placement, then run preflight_longrun.py.\n'
