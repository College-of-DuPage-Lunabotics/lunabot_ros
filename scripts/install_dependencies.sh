#!/bin/bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

install_ros_dependencies() {
    echo -e "\n------------------------ Install ROS Dependencies Using rosdep ------------------------ \n"
    cd "${WORKSPACE_DIR}"
    sudo rosdep init 2>/dev/null || true
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    
    # Remove ROS Humble grid_map packages that conflict with RTAB-Map's grid_map dependency
    echo -e "\n------------------------ Remove Conflicting grid_map Packages ------------------------ \n"
    sudo apt remove -y ros-humble-grid-map-core ros-humble-grid-map-ros ros-humble-grid-map-cv ros-humble-grid-map-msgs 2>/dev/null || true
}

install_camera_dependencies() {
    echo -e "\n------------------------ Install Camera Dependencies ------------------------ \n"
    sudo apt install -y ros-humble-realsense2-*
}

install_git_dependencies() {
    echo -e "\n------------------------ Install Git Dependencies and Pull LFS------------------------ \n"
    cd "${WORKSPACE_DIR}/src/lunabot_ros"
    sudo apt install git-lfs -y
    git lfs pull
}

install_sparkcan() {
    echo -e "\n------------------------ Add Repository and Install sparkcan ------------------------ \n"
    sudo add-apt-repository ppa:graysonarendt/sparkcan -y
    sudo apt update
    sudo apt install sparkcan -y
}

install_livox_sdk() {
    echo -e "\n------------------------ Install Livox SDK2 ------------------------ \n"
    cd /tmp
    if [ -d "Livox-SDK2" ]; then
        echo "Removing existing Livox-SDK2 directory..."
        rm -rf Livox-SDK2
    fi
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd Livox-SDK2
    mkdir build
    cd build
    # Use CMAKE_POLICY_VERSION_MINIMUM to handle newer CMake with older SDK
    cmake -DCMAKE_POLICY_VERSION_MINIMUM=3.5 .. && make -j$(nproc)
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
