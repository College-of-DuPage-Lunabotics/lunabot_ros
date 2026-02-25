#!/bin/bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SCRIPTS_DIR="${WORKSPACE_DIR}/src/lunabot_ros/scripts"

# Check if we're running with sudo
if [ "$EUID" -ne 0 ]; then
    echo "This script requires sudo privileges. Please run with sudo:"
    echo "  sudo ./setup_robot_services.sh"
    exit 1
fi

echo -e "\n------------------------ Install robot_upstart ------------------------ \n"
apt list --installed 2>/dev/null | grep -q ros-humble-robot-upstart
if [ $? -ne 0 ]; then
    echo "Installing ros-humble-robot-upstart..."
    apt install -y ros-humble-robot-upstart
else
    echo "robot_upstart already installed"
fi

echo -e "\n------------------------ Install CAN Setup Service ------------------------ \n"
if [ -f "${WORKSPACE_DIR}/src/lunabot_ros/lunabot_bringup/config/canable-setup.service" ]; then
    cp "${WORKSPACE_DIR}/src/lunabot_ros/lunabot_bringup/config/canable-setup.service" /etc/systemd/system/
    systemctl daemon-reload
    systemctl enable canable-setup.service
    echo "canable-setup.service installed and enabled"
else
    echo "Error: canable-setup.service not found"
    exit 1
fi

echo -e "\n------------------------ Install Launch Manager Service Using robot_upstart ------------------------ \n"
# Source ROS and workspace, run installer directly
source /opt/ros/humble/setup.bash
source ${WORKSPACE_DIR}/install/setup.bash
# Pass workspace directory via environment variable
LUNABOT_WS="${WORKSPACE_DIR}" python3 ${SCRIPTS_DIR}/install_launch_manager_service.py 0

echo -e "\n------------------------ Install systemd Network Wait Override ------------------------ \n"
# Create drop-in directory and install network-wait override
mkdir -p /etc/systemd/system/lunabot-launch-manager.service.d
if [ -f "${WORKSPACE_DIR}/src/lunabot_ros/lunabot_bringup/config/network-wait.conf" ]; then
    cp "${WORKSPACE_DIR}/src/lunabot_ros/lunabot_bringup/config/network-wait.conf" /etc/systemd/system/lunabot-launch-manager.service.d/
    echo "Installed systemd override to wait for network-online.target"
else
    echo "Warning: network-wait.conf not found"
fi

# Complete the installation
systemctl daemon-reload
systemctl enable lunabot-launch-manager

echo ""
echo "Services installed and enabled."
echo ""
echo "To start services now:"
echo "  sudo systemctl start canable-setup.service"
echo "  sudo systemctl start lunabot-launch-manager"
echo ""
echo "To check status:"
echo "  sudo systemctl status canable-setup.service"
echo "  sudo systemctl status lunabot-launch-manager"
echo ""
echo "To view logs:"
echo "  journalctl -u canable-setup.service"
echo "  journalctl -u lunabot-launch-manager -f"
echo ""
