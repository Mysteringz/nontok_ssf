#!/bin/bash
# Keeps the voice-called mission running forever in the container; restarts it 3 s after any crash.
# Start: docker exec -d nontok-dumptest bash /nontok/tools/mission_service.sh   (log: /tmp/mission.log)
# Stop:  docker exec nontok-dumptest pkill -f mission_service.sh; docker exec nontok-dumptest pkill -f mission_run.py
source /opt/ros/humble/setup.bash
source /nontok/ros2_ws/install/setup.bash
cd /nontok/ros2_ws
while true; do
    echo "=== mission service start $(date)" >> /tmp/mission.log
    python3 -u /nontok/tools/mission_run.py >> /tmp/mission.log 2>&1
    echo "=== mission_run exited ($?) $(date); restarting in 3 s" >> /tmp/mission.log
    sleep 3
done
