#!/bin/bash
# Switch the robot's Mega between the two firmwares. Runs ON THE JETSON (Mega on /dev/nontok_mega).
#   tools/mega_firmware.sh probe   teammate's HARDWARE_PROBE_2 (Nontok repo; their probe dashboard)
#   tools/mega_firmware.sh ssf     nontok_ssf (tools/wheel_bench.py, the ROS bridge)
#   tools/mega_firmware.sh which   report which one runs (resets the Mega, writes nothing)
# The Mega holds one program: only the tools of the firmware on it work. Hex files come from
# tools/build_mega_hex.sh on the Mac (named after their commits) in $MEGA_HEX_DIR
# (default ~/Documents/David/firmware) as probe_current.hex and ssf_current.hex.
# Each switch: the port must be free; avrdude writes and verifies (probe ~30 s, ssf ~7 s); the boot
# line is read back to confirm. Nothing moves: both firmwares start with the motors off.
# The probe dashboard runs on the Mac (Python >= 3.7): use Bluetooth, or move the USB cable.
set -euo pipefail
HEX_DIR=${MEGA_HEX_DIR:-$HOME/Documents/David/firmware}
PORT=${MEGA_PORT:-/dev/nontok_mega}

identify() {
  python3 - "$PORT" <<'EOF'
import sys, time, serial
s = serial.Serial(sys.argv[1], 115200, timeout=0.1)   # opening resets the Mega
end, text = time.time() + 8, b''
while time.time() < end:
    text += s.read(512)
    if b'READY NONTOK-SSF' in text or b'HARDWARE_PROBE' in text:
        break
if b'READY NONTOK-SSF' in text:
    s.write(b'!')
s.close()
if b'READY NONTOK-SSF' in text:
    line = [l for l in text.split(b'\n') if l.startswith(b'READY')][0].decode().strip()
    print('ssf   (' + line + ')')
elif b'HARDWARE_PROBE' in text:
    print('probe (HARDWARE_PROBE_2)')
else:
    print('unknown: ' + repr(text[:120]))
    sys.exit(1)
EOF
}

port_free() {
  local dev; dev=$(readlink -f "$PORT") || { echo "No Mega: $PORT missing"; exit 1; }
  [ -e "$dev" ] || { echo "No Mega: $PORT missing"; exit 1; }
  if fuser "$dev" >/dev/null 2>&1; then
    echo "Port $dev is in use by:"; fuser -v "$dev" 2>&1 | tail -n +2
    echo "Close that program first (wheel_bench.py, ROS base node, a serial monitor)."; exit 1
  fi
}

case "${1:-}" in
  probe|ssf) HEX=$HEX_DIR/${1}_current.hex ;;
  which) port_free; identify; exit ;;
  *) sed -n 2,11p "$0"; exit 1 ;;
esac
[ -f "$HEX" ] || { echo "Missing $HEX: run tools/build_mega_hex.sh on the Mac"; exit 1; }
port_free
echo "Flashing $1 from $(readlink -f "$HEX") ..."
# A failed write must not pass: the old firmware of the same kind would still boot and identify.
if ! OUT=$(avrdude -q -p m2560 -c wiring -P "$PORT" -b 115200 -D -U "flash:w:$HEX:i" 2>&1); then
  echo "$OUT" | tail -n 8; echo "AVRDUDE FAILED: the Mega may hold a half-written program; run this again"; exit 1
fi
echo "$OUT" | grep -E "signature|bytes of flash (written|verified)" || true
echo "$OUT" | grep -q "bytes of flash verified" || { echo "No verify line from avrdude"; exit 1; }
GOT=$(identify) || { echo "Flashed, but the boot line was not recognised: $GOT"; exit 1; }
echo "Running: $GOT"
case "$GOT" in "$1"*) ;; *) echo "WRONG FIRMWARE RUNNING"; exit 1 ;; esac
echo "$(date '+%F %T') $1 $(basename "$(readlink -f "$HEX")")" >> "$HEX_DIR/flash_history.txt"
