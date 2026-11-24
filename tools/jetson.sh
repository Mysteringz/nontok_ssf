#!/bin/bash
# Runs the Nontok software on the Jetson, inside its ROS 2 container. Run this ON the Jetson,
# from the project folder (~/nontok). The container sees all devices (cameras, Mega, sound).
#
#   tools/jetson.sh build                 colcon build
#   tools/jetson.sh shell                 a shell with ROS and the workspace ready
#   tools/jetson.sh run <command...>      one command, e.g. run ros2 run nontok_base wheel_check
#   tools/jetson.sh launch [args]         the whole robot: nontok.launch.py (add fake:=true to simulate)
#   tools/jetson.sh start [args]          the same in the background; then: tools/jetson.sh logs
#   tools/jetson.sh camdemo [args]      cameras, wake word and dashboard only - no motors (camera_demo.launch.py)
#   tools/jetson.sh cameras               which camera is on which USB port
#   tools/jetson.sh stop                  stop a running launch
#
# For real runs free the CPU first:  sudo systemctl isolate multi-user.target   (stops the desktop)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SETUP='source /opt/ros/humble/setup.bash; [ -f install/setup.bash ] && source install/setup.bash'
run() {   # run <container name> <docker options> -- <shell command>
    local name="$1" opts="$2" command="$3"
    docker run --rm --name "$name" $opts --privileged --network host \
        -e ROS_LOCALHOST_ONLY=1 -v /dev:/dev -v "$ROOT":/nontok -w /nontok/ros2_ws nontok-dev \
        bash -c "$SETUP; $command"
}
case "${1:-}" in
    build)   run nontok-build "" "colcon build --symlink-install" ;;
    shell)   run "nontok-shell-$$" "-it" "exec bash" ;;
    run)     shift; run "nontok-run-$$" "-it" "$*" ;;
    launch)  shift; docker rm -f nontok >/dev/null 2>&1
             run nontok "-it" "colcon build --symlink-install && source install/setup.bash && ros2 launch nontok_bringup nontok.launch.py $*" ;;
    start)   shift; docker rm -f nontok >/dev/null 2>&1
             docker run -d --name nontok --privileged --network host -e ROS_LOCALHOST_ONLY=1 -v /dev:/dev \
                 -v "$ROOT":/nontok -w /nontok/ros2_ws nontok-dev bash -c \
                 "$SETUP; colcon build --symlink-install && source install/setup.bash && ros2 launch nontok_bringup nontok.launch.py $*" >/dev/null \
                 && echo 'started (tools/jetson.sh logs to watch, tools/jetson.sh stop to end)' ;;
    camdemo) shift; docker rm -f nontok >/dev/null 2>&1
             docker run -d --name nontok --privileged --network host -e ROS_LOCALHOST_ONLY=1 -v /dev:/dev \
                 -v "$ROOT":/nontok -w /nontok/ros2_ws nontok-dev bash -c \
                 "$SETUP; colcon build --symlink-install && source install/setup.bash && ros2 launch nontok_bringup camera_demo.launch.py $*" >/dev/null \
                 && echo 'started (tools/jetson.sh logs to watch, tools/jetson.sh stop to end)' ;;
    logs)    docker logs -f --tail 40 nontok ;;
    stop)    docker rm -f nontok ;;
    cameras) for d in /dev/video*; do
                 [ "$(cat /sys/class/video4linux/$(basename $d)/index 2>/dev/null)" = "0" ] || continue
                 port=$(udevadm info -q path -n $d | grep -o 'usb[0-9]*/[0-9.-]*/[0-9.-]*' | awk -F/ '{print $NF}')
                 echo "$d  port $port  $(cat /sys/class/video4linux/$(basename $d)/name)"
             done ;;
    *)       sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//' ;;
esac
