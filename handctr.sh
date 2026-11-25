#!/bin/bash
# Field dashboard and hand control for the robot, on this Mac.
#
#   ./handctr.sh          stop the voice mission, start the dashboard with hand control, open it here
#   ./handctr.sh view     dashboard only (cameras, detection, readings); the voice mission keeps running
#   ./handctr.sh stop     disarm, stop the dashboard and close the tunnel (the mission stays stopped)
#   ./handctr.sh resume   stop the dashboard and start the voice mission again (./run.sh on the Jetson)
#   ./handctr.sh log      follow the dashboard's log on the Jetson
#
# Dashboard: http://localhost:8090 - three camera views, the detection switch, live readings, the mission
# log, and driving with W/S (forward/back), A/D (side to side), Q/E (turn), Space (stop).
# Each start copies tools/field_dash.* and the camera nodes it relies on to the Jetson; when a camera node
# changed, the robot stack is restarted (only when the mission is stopped, i.e. not in "view").
#
# Settings (environment): NONTOK_JETSON     user@host            (default nvidia@100.70.176.88)
#                         NONTOK_SOCKS      SOCKS proxy to it    (default localhost:1055, the userspace
#                                           Tailscale; ignored if nothing listens there)
#                         NONTOK_CONTAINER  robot container      (default nontok-dumptest, as run.sh)
#                         NONTOK_DASH_PORT  local port           (default 8090)
set -u
cd "$(dirname "$0")"
JETSON="${NONTOK_JETSON:-nvidia@100.70.176.88}"
SOCKS="${NONTOK_SOCKS:-localhost:1055}"
CONTAINER="${NONTOK_CONTAINER:-nontok-dumptest}"
PORT="${NONTOK_DASH_PORT:-8090}"
URL="http://localhost:$PORT"
SOCKET="${TMPDIR:-/tmp}/nontok-handctr-ssh.sock"
SSH=(ssh -o ControlMaster=auto -o ControlPath="$SOCKET" -o ControlPersist=8h -o ConnectTimeout=40
     -o ServerAliveInterval=15 -o StrictHostKeyChecking=accept-new)
if nc -z "${SOCKS%:*}" "${SOCKS#*:}" 2>/dev/null; then
    SSH+=(-o "ProxyCommand=nc -X 5 -x $SOCKS %h %p")
fi
# Files the dashboard needs on the robot. The last two are camera nodes: changing them needs a stack restart.
FILES=(tools/field_dash.py tools/field_dash.html
       ros2_ws/src/nontok_localization/nontok_localization/tag_detector_node.py
       ros2_ws/src/nontok_perception/nontok_perception/tray_node.py
       ros2_ws/src/nontok_perception/nontok_perception/tray_logic.py)
NODE_FILES=3   # FILES from this index on are camera nodes

say()    { printf '\033[1m[handctr]\033[0m %s\n' "$*"; }
remote() { "${SSH[@]}" "$JETSON" "$@"; }
in_box() { remote "docker exec $CONTAINER bash -c 'source /opt/ros/humble/setup.bash; source /nontok/ros2_ws/install/setup.bash; $1'"; }
running() { [ "$(remote "docker inspect -f '{{.State.Running}}' $CONTAINER 2>/dev/null")" = "true" ]; }

stop_dash() {   # a driving dashboard disarms on exit; disarm again only if the voice mission is not driving
    remote "docker exec $CONTAINER pkill -INT -f field_dash.py" 2>/dev/null && sleep 1.5
    remote "docker exec $CONTAINER pkill -9 -f field_dash.py" 2>/dev/null
    if ! remote "docker exec $CONTAINER pgrep -f mission_run.py >/dev/null" 2>/dev/null; then
        in_box 'timeout 6 ros2 service call /base/arm std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1' || true
    fi
}

sync_files() {   # copy what changed; returns 0 if a camera node changed
    local changed=() node_changed=1 i local_sum remote_sums
    remote_sums=$(remote "cd ~/nontok && md5sum ${FILES[*]} 2>/dev/null")
    for i in "${!FILES[@]}"; do
        local_sum=$(md5 -q "${FILES[$i]}")
        if ! grep -q "^$local_sum  ${FILES[$i]}\$" <<<"$remote_sums"; then
            changed+=("${FILES[$i]}")
            [ "$i" -ge "$NODE_FILES" ] && node_changed=0
        fi
    done
    if [ ${#changed[@]} -gt 0 ]; then
        say "copying to the Jetson: ${changed[*]}"
        tar czf - "${changed[@]}" | remote 'cd ~/nontok && tar xzf -' || { say "copy failed"; exit 1; }
    fi
    return $node_changed
}

tunnel() {
    if curl -s -m 2 -o /dev/null "$URL/api/state"; then return 0; fi
    "${SSH[@]}" -O check "$JETSON" 2>/dev/null || "${SSH[@]}" -f -N "$JETSON" || return 1
    "${SSH[@]}" -O forward -L "$PORT:localhost:8090" "$JETSON" 2>/dev/null || true
    for _ in 1 2 3 4 5; do curl -s -m 2 -o /dev/null "$URL/api/state" && return 0; sleep 1; done
    say "port $PORT on this Mac does not reach the dashboard (in use by something else? set NONTOK_DASH_PORT)"
    return 1
}

start() {
    local mode="$1"
    say "connecting to $JETSON ..."
    remote true || { say "cannot reach the Jetson (Tailscale up? NONTOK_JETSON=$JETSON)"; exit 1; }
    local nodes_changed=1
    sync_files && nodes_changed=0
    if [ "$mode" = drive ]; then
        say "stopping the voice mission (two drivers on one robot is unsafe)"
        remote 'cd ~/nontok && ./run.sh stop' | tail -1
        if ! running || [ $nodes_changed -eq 0 ]; then
            say "starting the robot stack (camera nodes updated or stack down) ..."
            remote 'cd ~/nontok && ./run.sh stack' | tail -3
        fi
    else
        running || { say "the robot stack is not running: start it on the Jetson with ./run.sh (or use ./handctr.sh)"; exit 1; }
        [ $nodes_changed -eq 0 ] && say "NOTE: camera nodes updated - detection needs a stack restart (./handctr.sh, or ./run.sh on the Jetson)"
    fi
    stop_dash
    local flag=''; [ "$mode" = view ] && flag='--view'
    remote "docker exec -d $CONTAINER bash -c 'source /opt/ros/humble/setup.bash; source /nontok/ros2_ws/install/setup.bash; exec python3 -u /nontok/tools/field_dash.py $flag > /tmp/field_dash.log 2>&1'"
    for _ in $(seq 30); do
        remote "curl -s -m 2 -o /dev/null localhost:8090/api/state" && break
        sleep 1
    done
    remote "curl -s -m 2 -o /dev/null localhost:8090/api/state" || { say "the dashboard did not start:"; remote 'tail -20 /tmp/field_dash.log'; exit 1; }
    tunnel || { say "tunnel failed"; exit 1; }
    [ -n "${NONTOK_NO_BROWSER:-}" ] || open "$URL"
    if [ "$mode" = drive ]; then
        say "dashboard at $URL - press Arm motors, then W/S A/D Q/E (Space stops). Done: ./handctr.sh resume"
    else
        say "dashboard at $URL (view only; the voice mission keeps running). Close: ./handctr.sh stop"
    fi
}

case "${1:-drive}" in
    drive|"") start drive ;;
    view)     start view ;;
    stop)     stop_dash; "${SSH[@]}" -O exit "$JETSON" 2>/dev/null; say "dashboard stopped, robot disarmed (mission still stopped: ./handctr.sh resume)" ;;
    resume)   stop_dash; say "starting the voice mission ..."; remote 'cd ~/nontok && ./run.sh' | tail -4
              say "dashboard closed; the mission finds the dock first if the robot is not on it" ;;
    log)      remote 'tail -f /tmp/field_dash.log' ;;
    *)        sed -n '2,19p' "$0" | sed 's/^# \{0,1\}//' ;;
esac
