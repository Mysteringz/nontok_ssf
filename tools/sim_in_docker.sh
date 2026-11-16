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
# NONTOK_DOCKER_ARGS adds options to docker run, e.g. "--cpus 1.5" to imitate a slow computer.
set -u
ITEM="${1:-resistor}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
docker image inspect nontok-dev >/dev/null 2>&1 || docker build -t nontok-dev "$ROOT/docker" || exit 2
docker rm -f nontok-sim >/dev/null 2>&1
docker run -d --name nontok-sim ${NONTOK_DOCKER_ARGS:-} -e ITEM="$ITEM" -v "$ROOT":/nontok -w /nontok/ros2_ws nontok-dev bash -c '
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install || exit 1
    source install/setup.bash
    ros2 launch nontok_bringup nontok.launch.py fake:=true fake_item:=$ITEM' >/dev/null || exit 2
# Wait for the build to finish and the launch to start (about 15 s on a laptop, 2 min on the Jetson).
for _ in $(seq 150); do
    docker exec nontok-sim pgrep -f mission_node >/dev/null 2>&1 && break
    [ -z "$(docker ps -q -f name=nontok-sim)" ] && { docker logs nontok-sim 2>&1 | tail -20; exit 2; }
    sleep 2
done
docker exec nontok-sim bash -c '
    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 /nontok/tools/sim_mission.py --timeout 400'
RESULT=$?
if [ $RESULT -ne 0 ]; then
    echo '--- warnings and errors from the launch ---'
    docker logs nontok-sim 2>&1 | grep -E 'ERROR|Traceback|died|\[WARN\]' | grep -v foxglove | tail -30
fi
exit $RESULT
