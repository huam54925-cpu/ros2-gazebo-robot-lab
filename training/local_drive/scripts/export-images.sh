#!/usr/bin/env bash
set -euo pipefail
OUT=${1:?Usage: export-images.sh /path/on/data-disk/migration}
mkdir -p "$OUT"
# Never replace a completed migration bundle silently.
if [[ -e "$OUT/local-drive-images.tar" ]]; then echo 'Bundle already exists' >&2; exit 2; fi
docker save -o "$OUT/local-drive-images.tar.part" \
  robot-sim:local-drive-20261011 robot-trainer:local-drive-cpu-v2
mv "$OUT/local-drive-images.tar.part" "$OUT/local-drive-images.tar"
(cd "$OUT" && sha256sum local-drive-images.tar > SHA256SUMS)
