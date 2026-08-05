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
