#!/bin/bash
# Shows what the robot is doing, live, in a browser on this Mac.
#
#   tools/mac_viz.sh              connect to the robot's dashboard and open it
#   tools/mac_viz.sh sim [item]   first start a SIMULATED robot on the Jetson and run one mission
#                                 (nothing moves; item = bolt, resistor, pliers, wire_offcut ...)
#   tools/mac_viz.sh stop         close the connection (and stop a simulation started from here)
#
# The dashboard is part of the robot's software (tools/jetson.sh launch on the Jetson). This
# script only opens an SSH tunnel to it and the browser. It asks for the Jetson's password
# unless a key is installed:  ssh-copy-id -i ~/.ssh/id_ed25519.pub <user>@<jetson>
#
# Settings (environment):  NONTOK_JETSON  user@host of the Jetson   (default nvidia@100.70.176.88)
#                          NONTOK_SOCKS   SOCKS proxy to reach it   (default localhost:1055, the
#                                         userspace Tailscale; ignored if nothing listens there)
#                          NONTOK_VIZ_PORT local port               (default 8080)
set -u
JETSON="${NONTOK_JETSON:-nvidia@100.70.176.88}"
SOCKS="${NONTOK_SOCKS:-localhost:1055}"
PORT="${NONTOK_VIZ_PORT:-8080}"
SOCKET="${TMPDIR:-/tmp}/nontok-viz-ssh.sock"
SSH=(ssh -o ControlMaster=auto -o ControlPath="$SOCKET" -o ControlPersist=8h -o ConnectTimeout=20
     -o ServerAliveInterval=15 -o StrictHostKeyChecking=accept-new)
if nc -z "${SOCKS%:*}" "${SOCKS#*:}" 2>/dev/null; then
    SSH+=(-o "ProxyCommand=nc -X 5 -x $SOCKS %h %p")
fi
URL="http://localhost:$PORT"

alive()  { curl -s -m 2 -o /dev/null "$URL/state"; }
tunnel() {
    "${SSH[@]}" -O check "$JETSON" 2>/dev/null && return 0
    echo "Connecting to $JETSON ..."
    "${SSH[@]}" -f -N -L "$PORT:localhost:8080" "$JETSON"
}

case "${1:-open}" in
    stop)
        "${SSH[@]}" "$JETSON" 'docker rm -f nontok-viz-sim >/dev/null 2>&1' 2>/dev/null
        "${SSH[@]}" -O exit "$JETSON" 2>/dev/null
        echo "Closed." ;;
    sim)
        ITEM="${2:-bolt}"
        tunnel || exit 1
        echo "Starting a simulated robot on the Jetson (the mission begins after about two minutes) ..."
        "${SSH[@]}" "$JETSON" "cd ~/nontok && docker rm -f nontok-viz-sim >/dev/null 2>&1; \
            docker run -d --name nontok-viz-sim --network host -e ROS_LOCALHOST_ONLY=1 -v \$PWD:/nontok -w /nontok/ros2_ws nontok-dev bash -c \
            'source /opt/ros/humble/setup.bash; colcon build --symlink-install >/dev/null 2>&1; source install/setup.bash; \
             ros2 launch nontok_bringup nontok.launch.py fake:=true fake_item:=$ITEM & \
             sleep 20; python3 /nontok/tools/sim_mission.py --settle 60 --timeout 500; sleep 600' >/dev/null" || exit 1
        for _ in $(seq 90); do alive && break; sleep 2; done
        alive || { echo "The dashboard did not come up. Look at: ssh $JETSON docker logs nontok-viz-sim"; exit 1; }
        [ -n "${NONTOK_NO_BROWSER:-}" ] || open "$URL"; echo "Showing the simulated mission at $URL  (tools/mac_viz.sh stop when done)" ;;
    open|"")
        tunnel || exit 1
        if alive; then
            [ -n "${NONTOK_NO_BROWSER:-}" ] || open "$URL"; echo "Dashboard at $URL  (tools/mac_viz.sh stop to disconnect)"
        else
            echo "Connected, but the robot's software is not running on the Jetson."
            echo "  On the Jetson:  cd ~/nontok && tools/jetson.sh start        (the real robot)"
            echo "  Or from here:   tools/mac_viz.sh sim                        (a simulated mission)"
            exit 1
        fi ;;
    *)  sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//' ;;
esac
