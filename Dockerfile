# Use the ROS 2 Iron base image
FROM ros:iron

# Set environment variables for ROS and NVIDIA
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV DEBIAN_FRONTEND=noninteractive

# Update and install base dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-pip \
    git \
    curl \
    libgl1-mesa-glx \
    libegl1-mesa \
    libgles2-mesa \
    libxinerama1 \
    libxcursor1 \
    libxrandr2 \
    x11-apps \
    alsa-utils \
    nano \
    sudo \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


# Set up environment variables and append to .bashrc
RUN echo 'unset GTK_PATH' >> ~/.bashrc && \
    echo 'source /opt/ros/iron/setup.bash' >> ~/.bashrc && \
    echo 'source /workspace/lunabot_ws/install/setup.bash' >> ~/.bashrc

# Source .bashrc
RUN /bin/bash -c "source ~/.bashrc && env"

# Create the workspace directory
WORKDIR /workspace/lunabot_ws/src

# Clone your repository and initialize submodules
RUN git clone -b iron https://github.com/grayson-arendt/Lunabotics-2025.git && \
    cd Lunabotics-2025 && \
    git submodule update --init --recursive --remote

# Copy and make the installation script executable
WORKDIR /workspace/lunabot_ws/src/Lunabotics-2025/scripts
RUN chmod +x install_dependencies.sh && ./install_dependencies.sh

# Set MAKEFLAGS to limit build threads
ENV MAKEFLAGS="-j1"

# Build the ROS workspace
WORKDIR /workspace/lunabot_ws
RUN colcon build --symlink-install --cmake-args \
    -DRTABMAP_SYNC_MULTI_RGBD=ON \
    -DWITH_OPENCV=ON \
    -DWITH_APRILTAG=ON \
    -DWITH_OPENGV=OFF \
    --parallel-workers 1

# Set default command
CMD ["/bin/bash"]
