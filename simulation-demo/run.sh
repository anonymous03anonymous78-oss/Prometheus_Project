#!/usr/bin/env bash
# The Living Map v3 - Gazebo + ROS 2 nodes + the 2D dataflow window, side by side.
#   ./run.sh <scenario> [world] [camera] [speed]
#     scenario: nominal | writer_destroyed | beacon_knockout | false_positive | all
#     world:    auto (default) | fuel (SubT tiles) | basic (offline fallback galleries)
#     camera:   auto (default: the story camera) | manual (you move the Gazebo camera)
#     speed:    1 (default, real time) | 2 | 3 ... Gazebo real-time factor (as fast as your PC allows)
cd "$(dirname "$0")"
SCENARIO="${1:-nominal}"
WORLD="${2:-auto}"
CAMERA="${3:-auto}"
SPEED="${4:-1}"
case "$SCENARIO" in
  nominal|writer_destroyed|beacon_knockout|false_positive|all) ;;
  *) echo "usage: ./run.sh [nominal|writer_destroyed|beacon_knockout|false_positive|all] [auto|fuel|basic]"; exit 1;;
esac
case "$WORLD" in auto|fuel|basic) ;; *) echo "world must be auto, fuel or basic"; exit 1;; esac
case "$CAMERA" in auto|manual) ;; *) echo "camera must be auto or manual"; exit 1;; esac
case "$SPEED" in ''|*[!0-9.]*) echo "speed must be a number (1, 2, 1.5 ...)"; exit 1;; esac
[ -f install/setup.bash ] || { echo "Run ./setup.sh first."; exit 1; }
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash

# leftovers from a previous run would share topics with this one
pkill -f "ign gazebo" 2>/dev/null
pkill -f "living_map.dry_run" 2>/dev/null
pkill -f "parameter_bridge" 2>/dev/null
pkill -f "lib/living_map/" 2>/dev/null
sleep 1

export QT_QPA_PLATFORM=xcb      # Gazebo GUI is most reliable through X11 / XWayland
echo "== The Living Map: scenario '$SCENARIO' (world: $WORLD, speed x$SPEED) =="
echo "   2D dataflow window: http://localhost:8765   (Ctrl+C here stops everything)"
bash ./scripts/open_view.sh "$SCENARIO" &
exec ros2 launch living_map sim.launch.py scenario:="$SCENARIO" world:="$WORLD" camera:="$CAMERA" speed:="$SPEED"
