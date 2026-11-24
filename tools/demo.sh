#!/bin/bash
# A demo on this Mac with no robot: the whole software stack runs in Docker against a simulated
# robot, and the live dashboard opens in the browser. Nothing here touches the Jetson.
#
#   tools/demo.sh [item]     start it and run one mission (item = led, resistor, bolt, multimeter)
#   tools/demo.sh busy [item]   the same in a room with two tables and two people walking about
#   tools/demo.sh voice      with it running: say "Nontok" to this Mac (or press Enter) to call the robot,
#                            and hear its answers on this Mac's speakers. Ctrl-C to leave.
#   tools/demo.sh again      call the robot again (same item) without talking
#   tools/demo.sh stop       shut it down
#
# NONTOK_DEMO_PORT  local port of the dashboard (default 8081; 8080 is the Jetson viewer's)
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PORT="${NONTOK_DEMO_PORT:-8081}"
URL="http://localhost:$PORT"
mission() {
    docker exec nontok-sim bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash
        python3 /nontok/tools/sim_mission.py --timeout 400 --settle '"$1"
}
case "${1:-resistor}" in
    stop)   docker rm -f nontok-sim >/dev/null 2>&1; echo 'demo stopped' ;;
    voice)  [ -n "$(docker ps -q -f name=nontok-sim)" ] || { echo 'not running: start it with tools/demo.sh [item]'; exit 2; }
            VENV="$ROOT/.local/demo-venv"
            if [ ! -x "$VENV/bin/python3" ]; then
                echo 'first time: installing the wake-word libraries into .local/demo-venv ...'
                python3 -m venv "$VENV" && "$VENV/bin/pip" install -q numpy onnxruntime openwakeword sounddevice pyyaml || exit 2
            fi
            exec "$VENV/bin/python3" -u "$ROOT/tools/demo_voice.py" ;;
    again)  [ -n "$(docker ps -q -f name=nontok-sim)" ] || { echo 'not running: start it with tools/demo.sh [item]'; exit 2; }
            mission 1 ;;
    *)      SCENE=''
            [ "${1:-}" = busy ] && { SCENE='scene:=busy'; shift; }
            ITEM="${1:-resistor}"
            docker image inspect nontok-dev >/dev/null 2>&1 || docker build -t nontok-dev "$ROOT/docker" || exit 2
            docker rm -f nontok-sim >/dev/null 2>&1
            docker run -d --name nontok-sim -p "$PORT":8080 -e ITEM="$ITEM" -e SCENE="$SCENE" -v "$ROOT":/nontok -w /nontok/ros2_ws nontok-dev bash -c '
                source /opt/ros/humble/setup.bash
                colcon build --symlink-install || exit 1
                source install/setup.bash
                ros2 launch nontok_bringup nontok.launch.py fake:=true fake_item:=$ITEM $SCENE' >/dev/null || exit 2
            echo "starting the simulated robot (item: $ITEM) ..."
            for _ in $(seq 90); do
                curl -s -m 2 -o /dev/null "$URL/state" && break
                [ -z "$(docker ps -q -f name=nontok-sim)" ] && { docker logs nontok-sim 2>&1 | tail -20; exit 2; }
                sleep 2
            done
            echo "dashboard: $URL"
            [ -n "${NONTOK_NO_BROWSER:-}" ] || open "$URL"
            mission 12
            echo "with voice: tools/demo.sh voice      again: tools/demo.sh again      stop: tools/demo.sh stop" ;;
esac
