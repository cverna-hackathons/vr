#!/bin/bash
# Bring up the Rift DK2 as an extended display (run after every boot / replug).
# Idempotent: if the output already runs the right mode, rotation and position, it does nothing.
# Usage: dk2-setup.sh [OUTPUT] [--force]    (--force reapplies even when the check says it is fine)
# Exit status: 0 = display ready (either already set up or configured now), 1 = DK2 output not connected.
# Prerequisite: the EDID override (sudo tools/dk2-edid-override.sh, then replug HDMI) so the
# kernel stops hiding the DK2 as a "non-desktop" output.
#
# Mode: the DK2's native timings need 75 Hz = 164.98 MHz, which the Meteor Lake HDMI PHY rejects
# ("unsupported HDMI clock"). 148.5 MHz is supported, so we use 60 Hz at that clock while keeping
# the panel's native line rate (116 kHz) and vertical timing; extra time goes into horizontal blanking.
set -euo pipefail
OUT=HDMI-1
FORCE=0
for a in "$@"; do
  case "$a" in
    --force) FORCE=1 ;;
    *) OUT="$a" ;;
  esac
done
MODE=1080x1920_60b
ROTATE="${ROTATE:-left}"

xrandr --query | grep -q "^$OUT connected" || { echo "$OUT not connected (did you run the EDID override and replug?)" >&2; exit 1; }

# Is the setup already in place? Needs: the custom mode active ('*' in the mode list), the wanted rotation, and a
# landscape desktop rectangle placed to the right of eDP-1 (same top edge as the built-in screen, or at least to its right).
ready() {
  local q line active edp_right dk_x
  q=$(xrandr --query)
  line=$(printf '%s\n' "$q" | grep "^$OUT connected")
  printf '%s\n' "$line" | grep -q " $ROTATE (" || return 1
  active=$(printf '%s\n' "$q" | awk -v out="$OUT" '$1==out {f=1; next} /^[^ ]/ {f=0} f && /\*/ {print $1}')
  [ "$active" = "$MODE" ] || return 1
  dk_x=$(printf '%s\n' "$line" | sed -n 's/.* [0-9]*x[0-9]*+\([0-9]*\)+[0-9]* .*/\1/p')
  edp_right=$(printf '%s\n' "$q" | sed -n 's/^eDP-1 connected[^0-9]*\([0-9]*\)x[0-9]*+\([0-9]*\)+[0-9]* .*/\1 \2/p' | awk '{print $1 + 0}')
  [ -n "$dk_x" ] && [ -n "$edp_right" ] && [ "$dk_x" -ge "$edp_right" ] || return 1
}
if [ "$FORCE" = 0 ] && ready; then
  echo "$OUT already set up ($MODE, rotated $ROTATE); nothing to do."
  exit 0
fi
xrandr --newmode "$MODE" 148.5 1080 1113 1123 1280 1920 1921 1927 1933 +hsync -vsync 2>/dev/null || true
xrandr --addmode "$OUT" "$MODE"
# The DK2 panel is mounted rotated; ROTATE=left gives an upright 1920x1080 landscape desktop.
xrandr --output "$OUT" --mode "$MODE" --rotate "$ROTATE" --right-of eDP-1
