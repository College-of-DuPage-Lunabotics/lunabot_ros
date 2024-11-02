#!/bin/bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

install_ros_dependencies() {
    echo -e "\n------------------------ Install ROS Dependencies Using rosdep ------------------------ \n"
    cd "${WORKSPACE_DIR}"
    
    sudo rosdep init 2>/dev/null || true
    rosdep update
    rosdep install --from-paths src --rosdistro foxy --ignore-src -r -y
}

main() {
    install_ros_dependencies
}

main
