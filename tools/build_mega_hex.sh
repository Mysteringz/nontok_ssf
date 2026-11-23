#!/bin/bash
# Build both Mega firmwares ON THE MAC and copy them to the Jetson for tools/mega_firmware.sh.
#   tools/build_mega_hex.sh [path/to/Nontok]        (default: ../Nontok next to this repo)
#   JETSON=nvidia@host  MEGA_HEX_DIR=dir (on the Jetson, from its home)  DIRTY=1 (allow edits)
# ssf   = this repo, firmware/nontok_ssf_mega, env mega2560      -> ssf_<commit>.hex
# probe = Nontok repo, firmware/hardware_probe_mega, env mega2560_probe -> probe_<commit>.hex
# Refuses uncommitted firmware changes so every hex matches a commit (DIRTY=1 adds "-dirty").
# On the Jetson the newest of each becomes ssf_current.hex / probe_current.hex. Flash nothing.
set -euo pipefail
REPO=$(git -C "$(dirname "$0")" rev-parse --show-toplevel)
NONTOK=$(cd "${1:-$REPO/../Nontok}" && git rev-parse --show-toplevel)
JETSON=${JETSON:-nvidia@10.68.194.30}
HEX_DIR=${MEGA_HEX_DIR:-Documents/David/firmware}
SSH_OPTS=(-o ControlMaster=auto -o "ControlPath=$HOME/.ssh/cm/%C" -o ControlPersist=120)
mkdir -p "$HOME/.ssh/cm"

stamp() {  # <repo> <firmware dir(s) that go into the build...>
  local repo=$1; shift
  local id; id=$(git -C "$repo" rev-parse --short HEAD)
  if [ -n "$(git -C "$repo" status --porcelain -- "$@")" ]; then
    if [ "${DIRTY:-0}" = 1 ]; then id="$id-dirty"; else
      echo "Uncommitted changes in $repo ($*): commit them, or run with DIRTY=1" >&2; exit 1; fi
  fi
  echo "$id"
}

SSF_ID=$(stamp "$REPO" firmware/nontok_ssf_mega)
# The probe sketch includes ../../core_robot_mega headers, so all of firmware/ counts.
PROBE_ID=$(stamp "$NONTOK" firmware)
echo "ssf   = nontok_ssf $SSF_ID ($(git -C "$REPO" rev-parse --abbrev-ref HEAD))"
echo "probe = Nontok $PROBE_ID ($(git -C "$NONTOK" rev-parse --abbrev-ref HEAD))"

(cd "$REPO/firmware/nontok_ssf_mega" && pio run -s -e mega2560)
(cd "$NONTOK/firmware/hardware_probe_mega" && pio run -s -e mega2560_probe)
OUT=$(mktemp -d)
cp "$REPO/firmware/nontok_ssf_mega/.pio/build/mega2560/firmware.hex" "$OUT/ssf_$SSF_ID.hex"
cp "$NONTOK/firmware/hardware_probe_mega/.pio/build/mega2560_probe/firmware.hex" "$OUT/probe_$PROBE_ID.hex"

echo "Copying to $JETSON:$HEX_DIR ..."
ssh "${SSH_OPTS[@]}" "$JETSON" "mkdir -p $HEX_DIR"
scp -q "${SSH_OPTS[@]}" "$OUT/ssf_$SSF_ID.hex" "$OUT/probe_$PROBE_ID.hex" "$JETSON:$HEX_DIR/"
ssh "${SSH_OPTS[@]}" "$JETSON" "cd $HEX_DIR && ln -sfn ssf_$SSF_ID.hex ssf_current.hex && \
  ln -sfn probe_$PROBE_ID.hex probe_current.hex && ls -l ssf_current.hex probe_current.hex"
rm -rf "$OUT"
echo "Done. On the Jetson: ~/nontok/tools/mega_firmware.sh probe | ssf | which"
