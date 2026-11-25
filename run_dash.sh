#!/bin/bash
# Live view of the robot's stage, reasoning, localization and route, on this Mac at http://localhost:8095.
#
#   ./run_dash.sh          start (or reconnect) and open the page
#   ./run_dash.sh stop     stop the sender on the Jetson and close the tunnel
#   ./run_dash.sh log      follow the sender's log
#
# The mission (tools/mission_run.py) writes a ~1-2 KB snapshot five times a second; tools/live_dash.py on the
# Jetson host streams it to the browser (no pictures, no ROS). Each start copies the three files over. If the
# mission script changed, the mission is restarted - but only while it is idle (listening on the dock);
# during a run it is left alone and the page says it has no live data until the next ./run.sh.
# For cameras, detection and driving use ./handctr.sh instead.
#
# Settings (environment): NONTOK_JETSON (default nvidia@100.70.176.88), NONTOK_SOCKS (default localhost:1055,
#                         the userspace Tailscale), NONTOK_LIVE_PORT (local port, default 8095)
set -u
cd "$(dirname "$0")"
JETSON="${NONTOK_JETSON:-nvidia@100.70.176.88}"
SOCKS="${NONTOK_SOCKS:-localhost:1055}"
PORT="${NONTOK_LIVE_PORT:-8095}"
URL="http://localhost:$PORT"
SOCKET="${TMPDIR:-/tmp}/nontok-live-ssh.sock"
SSH=(ssh -o ControlMaster=auto -o ControlPath="$SOCKET" -o ControlPersist=8h -o ConnectTimeout=40
     -o ServerAliveInterval=15 -o StrictHostKeyChecking=accept-new)
nc -z "${SOCKS%:*}" "${SOCKS#*:}" 2>/dev/null && SSH+=(-o "ProxyCommand=nc -X 5 -x $SOCKS %h %p")
FILES=(tools/live_dash.py tools/live_dash.html tools/mission_run.py)

say()    { printf '\033[1m[live]\033[0m %s\n' "$*"; }
remote() { "${SSH[@]}" "$JETSON" "$@"; }

start() {
    say "connecting to $JETSON ..."
    remote true || { say "cannot reach the Jetson (Tailscale up? NONTOK_JETSON=$JETSON)"; exit 1; }
    local sums changed=() mission_changed=no f
    sums=$(remote "cd ~/nontok && md5sum ${FILES[*]} 2>/dev/null")
    for f in "${FILES[@]}"; do
        grep -q "^$(md5 -q "$f")  $f\$" <<<"$sums" && continue
        changed+=("$f"); [ "$f" = tools/mission_run.py ] && mission_changed=yes
    done
    if [ ${#changed[@]} -gt 0 ]; then
        say "copying: ${changed[*]}"
        tar czf - "${changed[@]}" | remote 'cd ~/nontok && tar xzf -' || { say "copy failed"; exit 1; }
    fi
    if [ $mission_changed = yes ]; then
        if remote "grep -v 'HW ok' /tmp/mission.log | tail -1 | grep -q LISTENING"; then
            say "mission script updated and the robot is idle on the dock: restarting the mission"
            remote 'cd ~/nontok && ./run.sh' | tail -2
        else
            say "NOTE: mission script updated but the robot is busy - live data starts after the next ./run.sh"
        fi
    fi
    remote "pkill -f '[t]ools/live_dash.py'"   # separate call: its own command line must not contain the name
    remote "cd ~/nontok && nohup python3 -u tools/live_dash.py --port 8095 > /tmp/live_dash.log 2>&1 < /dev/null &"
    for _ in $(seq 10); do remote "curl -s -m 2 -o /dev/null localhost:8095/layout" && break; sleep 1; done
    remote "curl -s -m 2 -o /dev/null localhost:8095/layout" || { say "the sender did not start:"; remote 'tail /tmp/live_dash.log'; exit 1; }
    if ! curl -s -m 2 -o /dev/null "$URL/layout"; then
        "${SSH[@]}" -O check "$JETSON" 2>/dev/null || "${SSH[@]}" -f -N "$JETSON" || exit 1
        "${SSH[@]}" -O forward -L "$PORT:localhost:8095" "$JETSON" 2>/dev/null
        for _ in 1 2 3 4 5; do curl -s -m 2 -o /dev/null "$URL/layout" && break; sleep 1; done
        curl -s -m 2 -o /dev/null "$URL/layout" || { say "port $PORT here does not reach it (in use? set NONTOK_LIVE_PORT)"; exit 1; }
    fi
    [ -n "${NONTOK_NO_BROWSER:-}" ] || open "$URL"
    say "live view at $URL  (./run_dash.sh stop to close)"
}

case "${1:-start}" in
    start|"") start ;;
    stop)     remote "pkill -f '[t]ools/live_dash.py'" 2>/dev/null; "${SSH[@]}" -O exit "$JETSON" 2>/dev/null; say "closed" ;;
    log)      remote 'tail -f /tmp/live_dash.log' ;;
    *)        sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//' ;;
esac
