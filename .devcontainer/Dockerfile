# Base image with ROS 2 perception packages
FROM ros:iron-perception

# Arguments for creating a user
ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID

# Install dependencies
RUN apt update -q \
    && apt upgrade -q -y \
    && apt install -y --no-install-recommends \
    python3-pip \
    xauth \
    curl \
    git \
    software-properties-common \
    build-essential \
    libgazebo-dev \
    ros-iron-rviz2 \
    ros-iron-joint-state-publisher-gui \
    ros-iron-robot-state-publisher \
    ros-iron-rplidar-ros \
    ros-iron-realsense2-* \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Add SparkCAN repository and install
RUN add-apt-repository ppa:graysonarendt/sparkcan -y \
    && apt update -q \
    && apt install -y sparkcan \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Create and switch to user
RUN groupadd -g $GID $USERNAME \
    && useradd -lm -u $UID -g $USERNAME -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USERNAME

# Create workspace and set up the src/Lunabotics-2025 structure
RUN mkdir -p /home/$USERNAME/ros2_ws/src/Lunabotics-2025
WORKDIR /home/$USERNAME/ros2_ws

# Copy configuration files
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/ros2_ws/install/setup.bash' >> /home/$USERNAME/.bashrc

# Initialize rosdep
USER root
RUN apt update -q \
    && apt install -y --no-install-recommends \
    python3-rosdep \
    && if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && apt clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

USER $USERNAME
RUN rosdep update

# Copy the project to src/Lunabotics-2025
COPY . /home/$USERNAME/ros2_ws/src/Lunabotics-2025

# Use rosdep to install ROS dependencies
WORKDIR /home/$USERNAME/ros2_ws
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the ROS workspace with custom arguments
RUN /bin/bash -c "source /opt/ros/iron/setup.bash && colcon build --symlink-install --cmake-args \
    -DRTABMAP_SYNC_MULTI_RGBD=ON \
    -DWITH_OPENCV=ON \
    -DWITH_APRILTAG=ON \
    -DWITH_OPENGV=OFF \
    -DCMAKE_SUPPRESS_DEVELOPER_WARNINGS=ON \
    -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 1"

CMD ["bash"]
