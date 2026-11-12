#!/bin/bash
# Records the topics needed to replay and tune localisation (PLAN.md section 13.7).
#   tools/record_bag.sh [name]
set -e
name="${1:-nontok_$(date +%Y%m%d_%H%M%S)}"
ros2 bag record -o "$name" \
  /wheel/odom /imu/data /cmd_vel /base/status \
  /range/left45 /range/right45 /range/front /range/left /range/right \
  /tags/front_left/detections /tags/front_right/detections /tags/rear/detections \
  /tag_pose /odometry/local /odometry/global /localization/status \
  /tray/state /mission/state /tf /tf_static
