#!/usr/bin/env bash
# Opens the 2D dataflow window and tiles it next to Gazebo (Gazebo left, dataflow right).
URL="http://localhost:8765"
MODE="${2:-gazebo}"
python3 - "$URL" <<'PY'
import sys, time, urllib.request
for _ in range(240):
    try:
        urllib.request.urlopen(sys.argv[1] + '/config', timeout=1); sys.exit(0)
    except Exception:
        time.sleep(0.5)
sys.exit(1)
PY
[ $? -eq 0 ] || { echo "[open_view] dataflow server did not start"; exit 1; }

# screen geometry (primary monitor)
GEO="$(xrandr --current 2>/dev/null | grep ' primary' | grep -o '[0-9]*x[0-9]*+[0-9]*+[0-9]*' | head -1)"
[ -z "$GEO" ] && GEO="$(xrandr --current 2>/dev/null | grep -o 'current [0-9]* x [0-9]*' | awk '{print $2"x"$4"+0+0"}')"
[ -z "$GEO" ] && GEO="1920x1080+0+0"
SW="${GEO%%x*}"; REST="${GEO#*x}"; SH="${REST%%+*}"; OFF="${REST#*+}"; OX="${OFF%%+*}"; OY="${OFF#*+}"
HALF=$((SW / 2)); H=$((SH - 60))

BROWSER=""
for b in google-chrome chromium chromium-browser; do command -v "$b" >/dev/null && { BROWSER="$b"; break; }; done
if [ -n "$BROWSER" ]; then
  "$BROWSER" --app="$URL" --new-window --no-first-run --no-default-browser-check \
    --user-data-dir=/tmp/living_map_view --window-position=$((OX + HALF)),$OY \
    --window-size=$HALF,$H >/dev/null 2>&1 &
elif command -v firefox >/dev/null; then
  MOZ_ENABLE_WAYLAND=0 firefox --new-window "$URL" >/dev/null 2>&1 &
else
  xdg-open "$URL" >/dev/null 2>&1 &
fi

# tile: Gazebo on the left half, dataflow window on the right half
if command -v wmctrl >/dev/null; then
  for _ in $(seq 1 60); do
    GZ="$(wmctrl -l 2>/dev/null | awk '/ Gazebo$/{print $1; exit}')"
    VW="$(wmctrl -l 2>/dev/null | awk '/Living Map/{print $1; exit}')"
    if [ -n "$VW" ] && { [ -n "$GZ" ] || [ "$MODE" = "nogazebo" ]; }; then break; fi
    sleep 1
  done
  if [ -n "$GZ" ]; then
    wmctrl -i -r "$GZ" -b remove,maximized_vert,maximized_horz 2>/dev/null
    wmctrl -i -r "$GZ" -e 0,$OX,$OY,$HALF,$H 2>/dev/null
  fi
  if [ -n "$VW" ]; then
    wmctrl -i -r "$VW" -b remove,maximized_vert,maximized_horz 2>/dev/null
    wmctrl -i -r "$VW" -e 0,$((OX + HALF)),$OY,$HALF,$H 2>/dev/null
  fi
fi
echo "[open_view] If the windows are not side by side: click Gazebo + press Super+Left, click the dataflow window + press Super+Right."
