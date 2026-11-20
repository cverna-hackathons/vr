#!/bin/bash
# Undo tools/dk2-setup.sh (+ the EDID override) after taking the headset off: switch the DK2 output off, delete its custom mode,
# turn the external monitor back on above the built-in screen, and clear the EDID override so the fake "OVR/OVX" monitor goes away.
# Usage: tools/dk2-teardown.sh [OUTPUT] [EXTERNAL]     (defaults: HDMI-1 and the first connected output that is not eDP/the DK2)
# The last step needs root (sudo asks for your password / fingerprint); skip it with NO_SUDO=1 (the monitor then stays listed until reboot).
set -uo pipefail
OUT="${1:-HDMI-1}"
HERE="$(cd "$(dirname "$0")" && pwd)"
MODE=1080x1920_60b

# 1. DK2 output off, custom mode removed.
if xrandr --query | grep -q "^$OUT "; then
  xrandr --output "$OUT" --off
  xrandr --delmode "$OUT" "$MODE" 2>/dev/null || true
  xrandr --rmmode "$MODE" 2>/dev/null || true
  echo "$OUT switched off."
fi

# 2. Bring the external monitor back (above the built-in screen, as the desktop was before).
EXT="${2:-$(xrandr --query | awk -v dk="$OUT" '/ connected/ && $1 != dk && $1 !~ /^eDP/ {print $1; exit}')}"
if [ -n "$EXT" ]; then
  xrandr --output eDP-1 --auto --output "$EXT" --auto --primary --above eDP-1
  echo "$EXT enabled above eDP-1."
else
  xrandr --output eDP-1 --auto --primary
  echo "No external monitor connected; built-in screen only."
fi

# 3. Drop the EDID override, so HDMI-A-1 no longer pretends a monitor is attached.
if [ "${NO_SUDO:-0}" != 1 ]; then
  CONN="HDMI-A-1"
  [ "$OUT" = "HDMI-1" ] || CONN="$OUT"
  sudo "$HERE/dk2-edid-override.sh" --clear "$CONN" || echo "Override not cleared (run: sudo tools/dk2-edid-override.sh --clear)." >&2
fi
xrandr --query | grep -E "^[A-Za-z-]+[0-9-]* (connected|disconnected)" | cut -c1-90
