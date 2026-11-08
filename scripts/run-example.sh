#!/usr/bin/env bash
set -euo pipefail
AIRHUNT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$AIRHUNT_ROOT/scripts/env.sh" python "$AIRHUNT_ROOT/examples/cabinlake_umbrella/run.py" "$@"
