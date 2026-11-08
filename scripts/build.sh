#!/usr/bin/env bash
set -euo pipefail
AIRHUNT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$AIRHUNT_ROOT/scripts/env.sh" bash -c '
  cd "$AIRHUNT_ROOT/ros_ws"
  catkin config --extend "$AIRHUNT_RUNTIME" --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE="$AIRHUNT_RUNTIME/bin/python" -DCATKIN_ENABLE_TESTING=OFF
  catkin build --no-status -j 6
'
