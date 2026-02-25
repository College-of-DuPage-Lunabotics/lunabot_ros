#!/bin/bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
CONFIG_DIR="${WORKSPACE_DIR}/src/lunabot_ros/lunabot_bringup/config"

# Check if we're running with sudo
if [ "$EUID" -ne 0 ]; then
    echo "This script requires sudo privileges. Please run with sudo:"
    echo "  sudo ./setup_robot_services.sh"
    exit 1
fi

echo -e "\n------------------------ Install CAN Setup Service ------------------------ \n"
if [ -f "${CONFIG_DIR}/canable-setup.service" ]; then
    cp "${CONFIG_DIR}/canable-setup.service" /etc/systemd/system/
    systemctl daemon-reload
    systemctl enable canable-setup.service
    echo "canable-setup.service installed and enabled"
else
    echo "Error: ${CONFIG_DIR}/canable-setup.service not found"
    exit 1
fi

echo -e "\n------------------------ Install Launch Manager Service ------------------------ \n"
if [ -f "${CONFIG_DIR}/lunabot-launch-manager.service" ]; then
    cp "${CONFIG_DIR}/lunabot-launch-manager.service" /etc/systemd/system/
    systemctl daemon-reload
    systemctl enable lunabot-launch-manager.service
    echo "lunabot-launch-manager.service installed and enabled"
else
    echo "Error: ${CONFIG_DIR}/lunabot-launch-manager.service not found"
    exit 1
fi

echo ""
echo "Services installed and enabled. They will start on next boot."
echo ""
echo "To start services now:"
echo "  sudo systemctl start canable-setup.service"
echo "  sudo systemctl start lunabot-launch-manager.service"
echo ""
echo "To check status:"
echo "  sudo systemctl status canable-setup.service"
echo "  sudo systemctl status lunabot-launch-manager.service"
echo ""
echo "To view logs:"
echo "  journalctl -u canable-setup.service"
echo "  journalctl -u lunabot-launch-manager.service -f"
echo ""
