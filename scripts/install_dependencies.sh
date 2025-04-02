#!/bin/bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

install_ros_dependencies() {
    echo -e "\n------------------------ Install ROS Dependencies Using rosdep ------------------------ \n"
    cd "${WORKSPACE_DIR}"
    sudo rosdep init 2>/dev/null || true
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
}

install_sensor_dependencies() {
    echo -e "\n------------------------ Install Sensor Dependencies ------------------------ \n"
    sudo apt install -y ros-humble-rplidar-ros ros-humble-realsense2-*
    sudo wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | sudo bash
}

install_git_dependencies() {
    echo -e "\n------------------------ Install Git Dependencies and Pull LFS------------------------ \n"
    cd "${WORKSPACE_DIR}/src/Lunabotics-2025"
    sudo apt install git-lfs -y
    git lfs pull
}

install_sparkcan() {
    echo -e "\n------------------------ Add Repository and Install sparkcan ------------------------ \n"
    sudo add-apt-repository ppa:graysonarendt/sparkcan -y
    sudo apt update
    sudo apt install sparkcan -y
}

main() {
    install_sensor_dependencies
    install_git_dependencies
    install_sparkcan
    install_ros_dependencies
}

main
