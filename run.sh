#!/bin/bash
# Nontok: start everything for a full voice-called run, on the Jetson (outside the container).
#
#   ./run.sh            start (or restart) the robot stack and the mission; the robot must be on the dock
#   ./run.sh test N     same, then N unattended test loops (no wake word / item; mechanical, electrical, ...)
#   ./run.sh stack      start (or restart) only the robot stack and wait until it answers - no mission
#   ./run.sh stop       stop the mission, disarm, voice off (the stack keeps running)
#   ./run.sh down       stop the mission and the whole stack
#   ./run.sh status     what is running and whether the hardware answers
#   ./run.sh log        follow the mission log (Ctrl-C to leave)
#
# Options: NOT_DOCKED=1 ./run.sh   the robot is not on the dock: it finds the dock first.
# The mission (tools/mission_run.py) listens for "Nontok", fetches the item at the pickup, delivers it to its
# station, comes back to the dock and listens again, forever; tools/mission_service.sh restarts it on a crash.
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"

CONTAINER=${NONTOK_CONTAINER:-nontok-dumptest}
IMAGE=${NONTOK_IMAGE:-nontok-dev}
LOG=/tmp/mission.log

say() { printf '\033[1m[run]\033[0m %s\n' "$*"; }
in_box() { docker exec "$CONTAINER" bash -c "source /opt/ros/humble/setup.bash; source /nontok/ros2_ws/install/setup.bash; cd /nontok/ros2_ws; $1"; }
running() { [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = "true" ]; }

stop_mission() {
    running || return 0
    docker exec "$CONTAINER" pkill -9 -f mission_service.sh 2>/dev/null || true
    docker exec "$CONTAINER" pkill -9 -f mission_run.py 2>/dev/null || true
    in_box 'timeout 3 ros2 topic pub -t 5 -r 10 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1;
            timeout 6 ros2 service call /base/arm std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1;
            timeout 6 ros2 service call /voice/enable std_srvs/srv/SetBool "{data: false}" >/dev/null 2>&1' || true
    say "mission stopped, robot disarmed"
}

start_stack() {
    say "starting the robot stack (container $CONTAINER)"
    docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
    docker run -d --name "$CONTAINER" --privileged --network host -e ROS_LOCALHOST_ONLY=1 \
        -v /dev:/dev -v /tmp:/tmp -v "$PWD":/nontok -w /nontok/ros2_ws "$IMAGE" bash -c '
        source /opt/ros/humble/setup.bash; source install/setup.bash
        (ros2 launch nontok_bringup base.launch.py &)
        (ros2 launch nontok_bringup localization.launch.py cameras:=true &)
        (ros2 run nontok_perception tray_node --ros-args --params-file \
            $(ros2 pkg prefix nontok_perception)/share/nontok_perception/config/tray.yaml > /tmp/tray.log 2>&1 &)
        (ros2 run nontok_voice wake_node --ros-args --params-file \
            $(ros2 pkg prefix nontok_voice)/share/nontok_voice/config/voice.yaml > /tmp/wake.log 2>&1 &)
        exec ros2 run nontok_docking dock_server --ros-args --params-file \
            $(ros2 pkg prefix nontok_docking)/share/nontok_docking/config/docking.yaml' >/dev/null
}

hw_report() {   # one line per device; exit status 0 only if all answer
    in_box 'python3 - <<"EOF"
import time, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from nontok_interfaces.msg import TagDetectionArray
rclpy.init(); n = Node("run_sh_check"); seen = {}
n.create_subscription(Range, "range/front", lambda m: seen.__setitem__("mega", "front %.2f m" % m.range), 5)
for c in ("front_left", "front_right"):
    n.create_subscription(TagDetectionArray, "tags/%s/detections" % c,
                          lambda m, c=c: seen.__setitem__(c, "tags %s" % sorted(d.id for d in m.detections)), 5)
end = time.time() + 4
while time.time() < end and len(seen) < 3: rclpy.spin_once(n, timeout_sec=0.1)
names = dict(n.get_service_names_and_types())
seen["dock_server"] = "ok" if "/dock_to_tags/_action/send_goal" in names else None
seen["wake_word"] = "ok" if "/voice/enable" in names else None
seen["tray"] = "ok" if "/tray/classify" in names else None
bad = [k for k in ("mega", "front_left", "front_right", "dock_server", "wake_word", "tray") if not seen.get(k)]
for k in ("mega", "front_left", "front_right", "dock_server", "wake_word", "tray"):
    print("  %-12s %s" % (k, seen.get(k) or "NOT ANSWERING"))
raise SystemExit(1 if bad else 0)
EOF' 2>/dev/null
}

wait_ready() {
    say "waiting for the hardware and nodes (up to 3 min; the tray model takes ~1 min to load)"
    for i in $(seq 1 18); do
        sleep 10
        if hw_report >/tmp/run_sh_hw.txt 2>&1; then cat /tmp/run_sh_hw.txt; say "all up after $((i * 10)) s"; return 0; fi
    done
    cat /tmp/run_sh_hw.txt
    say "not everything answered - starting anyway: the mission restarts silent nodes / resets their USB itself"
}

start_mission() {
    local test_loops=${1:-0}
    local where=dock
    [ "${NOT_DOCKED:-0}" = "1" ] && where=away
    # state files are root-owned (written in the container), so write them there
    docker exec "$CONTAINER" bash -c "echo $where > /tmp/nontok_where; rm -f /tmp/nontok_test
                                      [ $test_loops -gt 0 ] && echo $test_loops > /tmp/nontok_test; true"
    docker exec "$CONTAINER" bash -c ": > $LOG"
    docker exec -d "$CONTAINER" bash /nontok/tools/mission_service.sh
    sleep 12
    grep -v Warn "$LOG" | tail -3 || true
    if [ "$test_loops" -gt 0 ]; then
        say "running $test_loops unattended test loops, then listening for \"Nontok\". Log: ./run.sh log"
    else
        say "listening for \"Nontok\" - say it, then put the item in the tray when asked. Log: ./run.sh log"
    fi
}

start_all() {
    amixer -q -c Audio sset PCM 100% unmute 2>/dev/null || say "speaker not found (the mission carries on without it)"
    stop_mission
    start_stack
    wait_ready
    start_mission "${1:-0}"
}

case "${1:-start}" in
    start) start_all 0 ;;
    test)  start_all "${2:?how many test loops, e.g. ./run.sh test 4}" ;;
    stack) stop_mission; start_stack; wait_ready ;;
    stop)  stop_mission ;;
    down)  stop_mission; docker rm -f "$CONTAINER" >/dev/null 2>&1 || true; say "stack stopped" ;;
    status)
        if ! running; then say "stack not running (./run.sh to start)"; exit 0; fi
        say "mission processes: $(docker exec "$CONTAINER" bash -c 'ps -eo args | grep -E "mission_(run|service)" | grep -v grep | wc -l')"
        say "last known place: $(cat /tmp/nontok_where 2>/dev/null || echo unknown); test loops left: $(cat /tmp/nontok_test 2>/dev/null || echo 0)"
        hw_report || true
        grep -v Warn "$LOG" 2>/dev/null | grep -E "^ *[0-9.]+s  " | tail -5 || true ;;
    log)   tail -f "$LOG" | grep --line-buffered -v Warn ;;
    *)     sed -n '2,14p' "$0"; exit 1 ;;
esac
