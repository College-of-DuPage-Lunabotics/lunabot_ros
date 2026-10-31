#!/bin/bash
# Records a rosbag of a sim or real test for offline analysis. Same as the GUI Record button, the
# topic list lives in lunabot_bringup/launch/record_launch.py.
#
#   scripts/record_test.sh sim [label]     run inside the sim container
#   scripts/record_test.sh real [label]    run on the robot so raw lidar and cameras are recorded
#
# Bags go to lunabot_ws/src/bags/<date>_<mode>[_label]. Stop with Ctrl-C.

if [ "$1" != "sim" ] && [ "$1" != "real" ]; then
    echo "usage: $0 sim|real [label]"
    exit 1
fi
use_sim=$([ "$1" = "sim" ] && echo true || echo false)

exec ros2 launch lunabot_bringup record_launch.py use_sim:=${use_sim} "label:=$2"
