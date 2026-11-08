#!/usr/bin/env bash
set -euo pipefail
AIRHUNT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export AIRHUNT_ROOT
if [[ -f "$AIRHUNT_ROOT/.env" ]]; then
  set -a
  source "$AIRHUNT_ROOT/.env"
  set +a
fi
: "${AIRHUNT_RUNTIME:=$AIRHUNT_ROOT/.runtime}"
: "${MICROMAMBA:=micromamba}"
export AIRHUNT_RUNTIME MICROMAMBA
export MPLBACKEND=Agg OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4
export PYTHONPATH="$AIRHUNT_ROOT/perception:$AIRHUNT_ROOT/third_party"
exec "$MICROMAMBA" run -p "$AIRHUNT_RUNTIME" "$@"
