# Use the ROS 2 Iron simulation image
FROM ros:iron

# Set environment variables for ROS
ENV DEBIAN_FRONTEND=noninteractive

# Arguments for creating a user
ARG USERNAME=rosdev
ARG UID=1000
ARG GID=$UID
ARG GPU_SUPPORT=false
ARG GPU_TYPE="none" # Options: "nvidia", "none"

# Enable GPU support for NVIDIA based on GPU_TYPE argument
RUN if [ "$GPU_SUPPORT" = "true" ]; then \
      if [ "$GPU_TYPE" = "nvidia" ]; then \
        echo "Installing NVIDIA GPU support"; \
        apt-get update && apt-get install -y nvidia-container-toolkit && \
        export NVIDIA_VISIBLE_DEVICES=all && \
        export NVIDIA_DRIVER_CAPABILITIES=compute,utility,video; \
      else \
        echo "No GPU support enabled"; \
      fi; \
    fi

# Install base dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    build-essential \
    python3-pip \
    git \
    git-lfs \
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
    ros-iron-rplidar-ros \
    ros-iron-realsense2-* \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Create and switch to a non-root user
RUN groupadd -g $GID $USERNAME && \
    useradd -m -s /bin/bash -u $UID -g $GID $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME

# Set up environment variables and append to .bashrc
RUN echo 'unset GTK_PATH' >> ~/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_zenoh_cpp' >> ~/.bashrc && \
    echo 'source /opt/ros/iron/setup.bash' >> ~/.bashrc && \
    echo 'source /home/lunabot_ws/install/setup.bash' >> ~/.bashrc

# Create the workspace directory
WORKDIR /home/lunabot_ws/src

# Clone your repository and initialize submodules
RUN git clone -b iron https://github.com/grayson-arendt/Lunabotics-2025.git && \
    cd Lunabotics-2025 && \
    git submodule update --init --recursive --remote

# Add sparkcan repository and install
USER root
RUN add-apt-repository ppa:graysonarendt/sparkcan -y && \
    apt-get update && \
    apt-get install -y sparkcan && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Initialize and update rosdep
WORKDIR /home/lunabot_ws
USER $USERNAME
RUN /bin/bash -c "sudo apt-get update && source /opt/ros/iron/setup.bash && \
    sudo rosdep init 2>/dev/null || true && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    sudo apt-get clean && rm -rf /var/lib/apt/lists/*"

# Set default command
CMD ["/bin/bash"]
