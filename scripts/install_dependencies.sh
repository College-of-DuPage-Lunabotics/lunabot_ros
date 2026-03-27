#!/bin/bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

# Detect Ubuntu codename and map to ROS distro
UBUNTU_CODENAME=$(lsb_release -cs)
case "${UBUNTU_CODENAME}" in
    jammy)  ROS_DISTRO="humble" ;;
    noble)  ROS_DISTRO="jazzy"  ;;
    *)
        echo "ERROR: Unsupported Ubuntu release '${UBUNTU_CODENAME}'. Only jammy (22.04) and noble (24.04) are supported."
        exit 1
        ;;
esac

echo "Detected Ubuntu ${UBUNTU_CODENAME} - using ROS 2 ${ROS_DISTRO}"

install_ros_dependencies() {
    echo -e "\n------------------------ Install ROS Dependencies Using rosdep ------------------------ \n"
    cd "${WORKSPACE_DIR}"
    sudo rosdep init 2>/dev/null || true
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y

    # Remove grid_map packages that conflict with RTAB-Map's grid_map dependency
    echo -e "\n------------------------ Remove Conflicting grid_map Packages ------------------------ \n"
    sudo apt remove -y \
        ros-${ROS_DISTRO}-grid-map-core \
        ros-${ROS_DISTRO}-grid-map-ros \
        ros-${ROS_DISTRO}-grid-map-cv \
        ros-${ROS_DISTRO}-grid-map-msgs 2>/dev/null || true
}

install_camera_dependencies() {
    echo -e "\n------------------------ Install Camera Dependencies ------------------------ \n"
    sudo apt install -y ros-${ROS_DISTRO}-realsense2-*
}

install_git_dependencies() {
    echo -e "\n------------------------ Install Git Dependencies and Pull LFS------------------------ \n"
    cd "${WORKSPACE_DIR}/src/lunabot_ros"
    sudo apt install git-lfs -y
    git lfs pull
}

install_sparkcan() {
    echo -e "\n------------------------ Install sparkcan ------------------------ \n"
    sudo add-apt-repository ppa:graysonarendt/sparkcan -y
    sudo apt update
    sudo apt install sparkcan -y
}

install_livox_sdk() {
    echo -e "\n------------------------ Install Livox SDK2 ------------------------ \n"
    local SDK_DIR="/tmp/Livox-SDK2"
    if [ -d "${SDK_DIR}" ]; then
        echo "Removing existing Livox-SDK2 directory..."
        sudo rm -rf "${SDK_DIR}"
    fi
    git clone https://github.com/Livox-SDK/Livox-SDK2.git "${SDK_DIR}"

    # Patch the SDK for noble to fix compilation issues with newer compilers
    if [ "${UBUNTU_CODENAME}" = "noble" ]; then
        sed -i '1s|^|#include <cstdint>\n|' "${SDK_DIR}/sdk_core/comm/define.h"
        sed -i '1s|^|#include <cstdint>\n|' "${SDK_DIR}/sdk_core/logger_handler/file_manager.h"
    fi

    mkdir -p "${SDK_DIR}/build"
    cd "${SDK_DIR}/build"
    cmake ..
    make -j$(nproc)
    if [ $? -ne 0 ]; then
        echo "ERROR: Livox SDK2 build failed."
        exit 1
    fi
    sudo make install
    echo "Livox SDK2 installed successfully"
}

install_gui_dependencies() {
    echo -e "\n------------------------ Install GUI Dependencies (PyQt5 and OpenCV) ------------------------ \n"
    sudo apt install -y python3-pyqt5 python3-opencv
}

main() {
    install_camera_dependencies
    install_git_dependencies
    install_sparkcan
    install_livox_sdk
    install_gui_dependencies
    install_ros_dependencies
}

main
