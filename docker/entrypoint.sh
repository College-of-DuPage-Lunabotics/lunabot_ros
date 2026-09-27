#!/bin/bash
set -e

# The build volume is created by docker as root on first use
if [ ! -w "$HOME/lunabot_ws" ]; then
    sudo chown "$(id -u):$(id -g)" "$HOME/lunabot_ws"
fi

unset GTK_PATH
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.bash
if [ -f "$HOME/lunabot_ws/install/setup.bash" ]; then
    source "$HOME/lunabot_ws/install/setup.bash"
fi

exec "$@"
