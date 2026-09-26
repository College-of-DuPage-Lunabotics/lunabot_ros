FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_UID=1000
ARG USER_GID=1000
ARG RENDER_GID=992

# Dependencies and tools for building and running the Lunabot ROS workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git git-lfs sudo curl \
    software-properties-common python3-pip python3-rosdep python3-colcon-common-extensions \
    python3-pyqt5 python3-opencv python3-serial \
    fonts-ubuntu fonts-noto-core fonts-dejavu-core \
    libgl1-mesa-dri libgl1-mesa-glx libegl1 mesa-utils \
    ros-humble-xacro ros-humble-joint-state-publisher \
    ros-humble-gazebo-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# Same UI font as the Ubuntu desktop
RUN printf '%s\n' \
    '<?xml version="1.0"?>' \
    '<!DOCTYPE fontconfig SYSTEM "fonts.dtd">' \
    '<fontconfig>' \
    '  <alias><family>sans-serif</family><prefer><family>Ubuntu Sans</family><family>Ubuntu</family><family>Noto Sans</family></prefer></alias>' \
    '  <alias><family>monospace</family><prefer><family>DejaVu Sans Mono</family></prefer></alias>' \
    '</fontconfig>' > /etc/fonts/local.conf \
    && fc-cache -f

# SparkMax CAN library used by lunabot_teleop
RUN add-apt-repository -y ppa:graysonarendt/sparkcan \
    && apt-get update && apt-get install -y --no-install-recommends sparkcan \
    && rm -rf /var/lib/apt/lists/*

# Livox SDK2, required by livox_ros_driver2
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 \
    && cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2/build \
    && cmake --build /tmp/Livox-SDK2/build -j"$(nproc)" \
    && cmake --install /tmp/Livox-SDK2/build \
    && rm -rf /tmp/Livox-SDK2

# Workspace dependencies from the package.xml files. grid_map from apt conflicts with
# the copy RTAB-Map builds, so it is removed afterwards like install_dependencies.sh does.
COPY . /tmp/deps_src/lunabot_ros
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash \
    && apt-get update && rosdep update \
    && rosdep install --from-paths /tmp/deps_src --ignore-src -r -y \
    && apt-get remove -y ros-humble-grid-map-core ros-humble-grid-map-ros \
       ros-humble-grid-map-cv ros-humble-grid-map-msgs || true \
    && rm -rf /var/lib/apt/lists/* /tmp/deps_src

# Non-root user matching the host uid so files written into the mounted src stay yours.
# The render group is created with the host gid so /dev/dri/renderD128 is usable.
RUN groupadd -g ${RENDER_GID} render \
    && groupadd -g ${USER_GID} lunabot \
    && useradd -m -u ${USER_UID} -g ${USER_GID} -G sudo,video,render,dialout -s /bin/bash lunabot \
    && echo "lunabot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/lunabot

USER lunabot
RUN mkdir -p /home/lunabot/lunabot_ws
WORKDIR /home/lunabot/lunabot_ws

RUN echo 'unset GTK_PATH' >> ~/.bashrc \
    && echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc \
    && echo 'source /usr/share/gazebo/setup.bash' >> ~/.bashrc \
    && echo '[ -f ~/lunabot_ws/install/setup.bash ] && source ~/lunabot_ws/install/setup.bash' >> ~/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
