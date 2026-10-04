#!/usr/bin/env bash
# The Living Map v3 - one-time setup (Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress)
set -e
cd "$(dirname "$0")"
WS="$(pwd)"
ok()   { echo -e "  \e[32m✓\e[0m $*"; }
warn() { echo -e "  \e[33m!\e[0m $*"; }
die()  { echo -e "  \e[31m✗ $*\e[0m"; exit 1; }

echo "== The Living Map v3: setup =="
. /etc/os-release
[ "$VERSION_ID" = "22.04" ] && ok "Ubuntu $VERSION_ID" || warn "expected Ubuntu 22.04, found $VERSION_ID (continuing)"
[ -f /opt/ros/humble/setup.bash ] || die "ROS 2 Humble not found in /opt/ros/humble"
set +eu; source /opt/ros/humble/setup.bash; set -e
ok "ROS 2 $ROS_DISTRO"
command -v ign >/dev/null || die "'ign' command not found - is Gazebo Fortress installed?"
GZV="$(ign gazebo --versions 2>/dev/null | head -1)"
case "$GZV" in 6*) ok "Gazebo Fortress (ign gazebo $GZV)";; *) warn "ign gazebo version '$GZV' (expected 6.x Fortress)";; esac

echo "== Installing missing packages (sudo) =="
sudo apt-get update -qq
sudo apt-get install -y ros-humble-ros-gz-bridge python3-numpy python3-colcon-common-extensions \
  wmctrl x11-xserver-utils
ok "packages installed"

echo "== Downloading the Gazebo Fuel models (DARPA SubT tunnel tiles + Rescue Randy), once =="
FUEL="https://fuel.gazebosim.org/1.0/OpenRobotics/models"
FUEL_OK=1
for m in "Tunnel Tile 1" "Tunnel Tile 5" "Tunnel Tile 6" "Rescue Randy Sitting"; do
  if ign fuel download -u "$FUEL/$m" >/tmp/living_map_fuel.log 2>&1; then
    ok "$m"
  else
    warn "$m could not be downloaded (see /tmp/living_map_fuel.log)"; FUEL_OK=0
  fi
done
if [ "$FUEL_OK" = 1 ]; then
  ok "full world available (SubT tiles)"
else
  warn "run ./setup.sh again with internet for the SubT tiles; until then run.sh uses the offline world"
fi

echo "== Building workspace =="
colcon build --symlink-install --packages-select living_map
ok "build done"

echo "== Self-test: mission logic, scenario 'all' on the dry-run stand-in (about 4 minutes) =="
out="$(cd "$WS/src/living_map" && LM_MEMORY=off python3 -m living_map.dry_run --scenario all --fast --no-view --quiet 2>&1 | grep -E '^(phase|max est)' || true)"
if echo "$out" | grep -q mission_complete; then ok "all scenarios logic OK"; echo "$out" | sed 's/^/     /'; else die "self-test failed: $out"; fi

echo
echo "Setup complete. Run a scenario with:"
echo "   ./run.sh nominal | writer_destroyed | beacon_knockout | false_positive | all"
echo "Speed for recording the demo:  ./run.sh all auto auto 2"
echo "Full 5-scenario regression (about 20 minutes):  python3 src/living_map/test/test_scenarios.py"
