#!/usr/bin/env bash
# Flash tovez in the field through gauti, with no cable to the bench.
#
# gauti is the Pi riding on the robot: USB to the micro:bit, wifi to us. It
# exists so the robot can be reflashed where it stands, on battery, mid-run.
# Commanding still goes over the radio relay.
#
# Refuses to write any board whose DAPLink UID is not tovez's. The micro:bit
# MSD re-enumerates on every flash (sda -> sdb -> ...), so the volume is
# found by LABEL, never by a remembered device path.
#
#   src/tests/bench/field_flash.sh [path/to/MICROBIT.hex]
set -euo pipefail

HEX="${1:-MICROBIT.hex}"
HOST=ros@gauti
TOVEZ=9906360200052820a8fdb5e413abb276000000006e052820

[ -f "$HEX" ] || { echo "no such hex: $HEX" >&2; exit 1; }

# The hex must carry the version the build claims. An incremental build on
# /Volumes can leave CMake thinking nothing changed, regenerate
# version_generated.h, and then re-copy the PREVIOUS hex with a fresh
# timestamp -- so build.py prints a version the image does not contain, and
# the robot boots the old firmware while reporting success. Compare the two
# and refuse rather than flash a lie.
HEX_VER=$(python3 - "$HEX" <<'PYEOF'
import re, sys
buf = bytearray()
for line in open(sys.argv[1]):
    if len(line) < 9 or line[0] != ':':
        continue
    n, t = int(line[1:3], 16), int(line[7:9], 16)
    if t == 0:
        buf += bytes.fromhex(line[9:9 + 2 * n])
found = sorted({m.decode() for m in re.findall(rb'\d+\.\d{8}\.\d+', buf)})
print(found[0] if len(found) == 1 else "")
PYEOF
)
SRC_VER=$(sed -n 's/.*FIRMWARE_VERSION_STR "\(.*\)".*/\1/p' src/firm/types/version_generated.h)
if [ -n "$HEX_VER" ] && [ -n "$SRC_VER" ] && [ "$HEX_VER" != "$SRC_VER" ]; then
  echo "REFUSING: hex carries $HEX_VER but the tree says $SRC_VER -- stale build." >&2
  echo "          run: uv run python3 build.py --clean --robot-debug" >&2
  exit 1
fi
echo "-> hex version ${HEX_VER:-unknown}"

echo "-> copying $(basename "$HEX") ($(wc -c <"$HEX") bytes)"
scp -q "$HEX" "$HOST:/tmp/MICROBIT.hex"

ssh "$HOST" 'bash -s' <<'REMOTE'
set -e
TOVEZ=9906360200052820a8fdb5e413abb276000000006e052820
DEV=$(blkid -L MICROBIT 2>/dev/null || true)
[ -z "$DEV" ] && { echo "no MICROBIT volume mounted"; exit 1; }
sudo mkdir -p /mnt/microbit
sudo umount /mnt/microbit 2>/dev/null || true
sudo mount "$DEV" /mnt/microbit
SEEN=$(grep -i "Unique ID" /mnt/microbit/DETAILS.TXT | awk '{print $NF}' | tr -d '\r\n')
if [ "$SEEN" != "$TOVEZ" ]; then echo "REFUSING: [$SEEN] is not tovez"; exit 1; fi
echo "   $DEV verified tovez"
sudo cp /tmp/MICROBIT.hex /mnt/microbit/
sync
REMOTE

echo "-> waiting for reboot"
sleep 8
echo "   flashed; command it over the radio relay as usual"
