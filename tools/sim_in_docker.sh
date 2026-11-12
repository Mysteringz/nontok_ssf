#!/bin/bash
# Build the workspace and run one simulated mission in a ROS 2 Humble container.
# Needs Docker. No robot, cameras or microphone.
#
#   tools/sim_in_docker.sh [item]        item = a name from nontok_perception/config/items.yaml
#
# Exit code 0 = the robot went dock -> pickup -> station -> dock.
# The container is left running afterwards for a look around:
#   docker exec -it nontok-sim bash       (Foxglove: add -p 8765:8765 to the run line below)
#   docker rm -f nontok-sim               when finished
set -u
ITEM="${1:-resistor}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
docker image inspect nontok-dev >/dev/null 2>&1 || docker build -t nontok-dev "$ROOT/docker" || exit 2
docker rm -f nontok-sim >/dev/null 2>&1
docker run -d --name nontok-sim -e ITEM="$ITEM" -v "$ROOT":/nontok -w /nontok/ros2_ws nontok-dev bash -c '
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install || exit 1
    source install/setup.bash
    ros2 launch nontok_bringup nontok.launch.py fake:=true fake_item:=$ITEM' >/dev/null || exit 2
sleep 15   # the build
docker exec nontok-sim bash -c '
    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 /nontok/tools/sim_mission.py --timeout 400'
RESULT=$?
if [ $RESULT -ne 0 ]; then
    echo '--- warnings and errors from the launch ---'
    docker logs nontok-sim 2>&1 | grep -E 'ERROR|Traceback|died|\[WARN\]' | grep -v foxglove | tail -30
fi
exit $RESULT
