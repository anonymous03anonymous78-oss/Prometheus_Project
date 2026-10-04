#!/usr/bin/env bash
# Safety net: the complete mission + 2D dataflow window WITHOUT Gazebo (kinematic stand-in).
#   ./dry_run.sh <scenario> [speed]      e.g.  ./dry_run.sh all 2
cd "$(dirname "$0")"
SCENARIO="${1:-nominal}"
SPEED="${2:-1}"
export PYTHONPATH="$(pwd)/src/living_map:$PYTHONPATH"
bash ./scripts/open_view.sh "$SCENARIO" nogazebo &
exec python3 -m living_map.dry_run --scenario "$SCENARIO" --speed "$SPEED"
