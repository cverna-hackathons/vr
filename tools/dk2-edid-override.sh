#!/bin/bash
# The Linux kernel flags the Rift DK2's EDID (vendor OVR, product 0x0003) as "non-desktop",
# so X hides the output (kernel exposes no modes, xrandr says "disconnected").
# This script feeds the kernel a copy of the EDID with a different product code so the DK2
# appears as an ordinary extended monitor. Run as root; effect lasts until reboot/unplug of override.
#
# Usage: sudo tools/dk2-edid-override.sh [connector]      (default: HDMI-A-1)
#        sudo tools/dk2-edid-override.sh --clear [connector]
set -euo pipefail

CLEAR=0
if [ "${1:-}" = "--clear" ]; then CLEAR=1; shift; fi
CONN="${1:-HDMI-A-1}"

mount | grep -q debugfs || mount -t debugfs none /sys/kernel/debug
DIR=$(ls -d /sys/kernel/debug/dri/*/"$CONN" 2>/dev/null | head -n1 || true)
[ -n "$DIR" ] || { echo "No debugfs entry for connector $CONN" >&2; exit 1; }

# Force the kernel to re-read the EDID. trigger_hotplug can be denied (e.g. lockdown), so
# fall back to the sysfs "detect" write, then to asking for a manual replug.
reprobe() {
  echo 1 > "$DIR/trigger_hotplug" 2>/dev/null && return 0
  echo detect > /sys/class/drm/card*-"$CONN"/status 2>/dev/null && return 0
  echo "Could not force a reprobe; unplug and replug the DK2 HDMI cable." >&2
}

if [ "$CLEAR" = 1 ]; then
  echo reset > "$DIR/edid_override"
  reprobe
  echo "Override cleared on $CONN"; exit 0
fi

SYS=$(ls -d /sys/class/drm/card*-"$CONN" | head -n1)
TMP=$(mktemp)
python3 - "$SYS/edid" "$TMP" <<'PY'
import sys
e = bytearray(open(sys.argv[1], 'rb').read())
assert len(e) >= 128 and e[8:10] == bytes([0x3e, 0xd2]), "not an Oculus (OVR) EDID"
e[9] = 0xd8                               # manufacturer "OVR" (3ed2) -> "OVX" (3ed8); the kernel's non-desktop
e[10] = 0x04                              # quirk may match on vendor alone, so also change product 0x0003 -> 0x0004
e[127] = (-sum(e[:127])) & 0xff           # fix base-block checksum
open(sys.argv[2], 'wb').write(e)
PY
cat "$TMP" > "$DIR/edid_override"
rm -f "$TMP"
reprobe
echo "Override applied on $CONN. Check: xrandr"
