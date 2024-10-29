# Project Overview

This repository contains the software developed by the College of DuPage team for the 2024-2025 NASA Lunabotics competition. It is built for ROS 2 Humble on Ubuntu 22.04.

## System Components

**Computer**
- ASRock 4X4 BOX-8840U

**Sensors**
- RPLidar S3
- Intel RealSense D455 Depth Camera
- Intel RealSense D456 Depth Camera
  
**Hardware**
- REV Robotics NEO Vortex (x2)
- REV Robotics Spark Max (x2)
- REV Robotics Power Distribution Hub
- Turnigy 14.8V 8000mAh LiPo Battery
- Turnigy 14.8V 12000mAh LiPo Battery
- ODrive USB-CAN Adapter

## Installation

**Note: You will need to have already installed ROS 2 Humble before continuing with installation. The guide can be found [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Install both `ros-humble-desktop` and `ros-dev-tools`.**

#### (Optional) 1. Append lines to .bashrc

.bashrc is a script that runs everytime a new terminal window is opened and has various configurations, environment variables, and commands for setup. There is a bug in the VSCode terminal that will cause a symbol lookup error, so you have to unset the path variable using `unset GTK_path`. If you haven't already added `source /opt/ros/humble/setup.bash` to your .bashrc file, it simply runs the setup script for ROS 2 Humble.

```bash
echo 'unset GTK_PATH' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

This will permanently append these two lines to your .bashrc file, so there is no need to run it again. If you want to edit the file manually, use `nano ~/.bashrc` or `gedit ~/.bashrc` if you prefer a text editor GUI instead.

#### 2. Setup workspace and clone repository

```bash
mkdir -p ~/lunabot_ws/src
cd ~/lunabot_ws/src
git clone --recursive https://github.com/grayson-arendt/Lunabotics-2025.git
```
**If you have previously cloned this repository and do not see anything in the folders located in `lunabot_third_party`, run `git submodule update --init --recursive` inside the `Lunabotics-2025` folder to initialize the submodules.**

#### 3. Install dependencies

Run the installation script to install the required dependencies. `chmod +x` gives permission for the script to be executable.

```bash
cd ~/lunabot_ws/src/Lunabotics-2025/scripts
chmod +x install_dependencies.sh
./install_dependencies.sh
```

#### 4. Build the workspace

Building may take some time due to the external packages in `lunabot_third_party`. Various flags such as `-DRTABMAP_SYNC_MULTI_RGBD=ON` need to be set to enable extra features for RTAB-Map.

To avoid building the entire workspace all over again after the initial build if you make changes, use `colcon build --packages-select name_of_package` and choose the package that you made changes to for rebuilding. You can list multiple packages after the `--packages-select` flag. You only need to rebuild the workspace if you modify a `C++` file, the flag `--symlink-install` will automatically reflect the changes in `Python, URDF, Xacro, and YAML` files.

```bash
cd ~/lunabot_ws
colcon build --symlink-install --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DWITH_OPENCV=ON -DWITH_APRILTAG=ON -DWITH_OPENGV=OFF --parallel-workers 4 # Modify number as needed, this is how many packages are built concurrently
```

#### 5. (Optional) Set MAKEFLAG and Rebuild
If your computer keeps crashing while trying to build, `colcon build` may be trying to do too many things at once. Setting this flag to `-j1` limits each package's internal make jobs to 1 thread. You can either increase or reduce both this and `--parallel-workers`, increasing will make it build faster but may put more stress on your computer, leading to freezing.

```bash
export MAKEFLAGS="-j1" # Modify number as needed
```

Next, rebuild using the same commands in step **4. Build the workspace**.

## Simulating the Robot
**Note: The side of the robot without the bulldozer blade is considered the front of the robot, as the camera is able to see obstacles better on that side. Therefore, controls may seem reversed but it is intentional.**

There are two modes for simulating the robot: **manual** and **autonomous**. 

##### Manual Mode
This launches a teleop node for controlling the Gazebo robot with either a keyboard or Xbox controller along with Navigation2 and RTAB-Map, but does not launch the `localization_server` or `navigation_client`. In this mode, you can drive the robot around, map the arena, and play around with setting Nav2 goals in RViz2.

##### Autonomous Mode
This launches `localization_server` and `navigation_client` and will not allow the user to teleop the robot. Instead, it will follow the commands in the server and client and perform a one-cycle autonomy sequence. 

#### 1. Navigate to workspace and source setup

```bash
cd ~/lunabot_ws
source install/setup.bash
```
#### Open separate terminal windows and source the workspace setup for each next step:

#### 2. Visualize with RViz2

```bash
ros2 launch lunabot_bringup visualization_launch.py # orientation:=north (to specify a starting orientation in Gazebo: north, south, east, west, or random)
```

#### 3. Launch simulation

```bash
ros2 launch lunabot_bringup simulation_launch.py # teleop:=xbox (for Xbox controller) mode:=autonomous (to run in autonomous mode)
```

<p align="center">
  <img src="demo.png">
</p>

## Running the Physical Robot


### SSH Into Robot Computer

SSH (Secure Shell) allows you access another device over the network and run commands. The **host computer** is the computer that you are personally interfacing with, such as laptop. For any future steps that require being ran on the robot computer, you will need to be connected via SSH.


#### 1. Install and enable SSH server (host computer):
```bash
sudo apt update
sudo apt install openssh-server

sudo systemctl start ssh
sudo systemctl enable ssh
```

#### 2. Create SSH-key (host computer)

```bash
ssh-keygen
```

#### 3. Get username and IP address (robot computer)

```bash
whoami
```
This will return the username of the robot computer, although you can also see the username just by looking at the terminal. It is the first name before the @, for example, the username would be `grayson` for `grayson@NUC`.

Next, get the IP address:
```bash
hostname -I
```

The IP address is the first set of numbers in the list.

#### 4. Establish SSH connection (host computer)

Using the username and IP address from the previous step, now you can connect to the robot computer. It may look something like this for example:

```bash
ssh asrock@192.168.10.1 # (General format: username@ip_address)
```
 It will ask you if you are sure you want to connect, type `yes`. Then, confirm by typing in the robot computer's password. 

### Configure Device Permissions 

#### 1. Add user to dialout group then restart (robot computer)

```bash
sudo usermod -a -G dialout $USER
```

Use `ls /dev/ttyUSB*` to identify device numbers if the lidars are disconnected and reconnected, then adjust the lidar `"serial_port"` parameters in `real_launch.py` accordingly.

#### 2. Setup camera udev rules (robot computer)

```bash
cd ~/lunabot_ws/src/Lunabotics-2025/scripts
chmod +x setup_udev_rules.sh
sudo ./setup_udev_rules.sh
```

Make sure all cameras are unplugged while setting up the udev rules.

### Running Launch Files

#### 1. Initialize SocketCAN (robot computer)

```bash
cd ~/lunabot_ws/src/Lunabotics-2025/scripts/
chmod +x canable_start.sh
./canable_start.sh
```

#### 2. Source workspace setup (both host and robot computer)

```bash
cd ~/lunabot_ws
source install/setup.bash
```

#### Open separate terminal windows and source the workspace setup for each next step:

#### 3. Visualize with RViz2 (host computer)

```bash
ros2 launch lunabot_bringup visualization_launch.py visualization_mode:=real
```

#### 4. Launch the real robot (robot computer)

```bash
ros2 launch lunabot_bringup real_launch.py # mode:=autonomous (to run in autonomous mode)
```

## Project Structure

**lunabot_bringup**: This package contains launch files to bring up autonomy nodes, Gazebo simulation, and real world hardware.
- **launch**
  - **real_launch.py**: Launches the required nodes for bringing up the physical robot hardware and sensors, along with manual control and/or autonomy nodes.
  - **simulation_launch.py**: Launches the required nodes for simulating robot autonomy in Gazebo.
  - **visualization_launch.py**: Launches RViz2 and Gazebo to visualize the robot and its sensor data.

**lunabot_config**: This package contains configuration files for Navigation2 behavior trees, RViz2 settings, and various 
- **behavior_trees**
  - **navigate_through_poses_w_replanning_and_recovery.xml**: A behavior tree used with Navigation2 to implement behaviors like goal replanning and recovery for NavigateThroughPoses action.
  - **nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml**: A behavior tree used with Navigation2 to only replan when path becomes invalid to prevent Navigation2 from repeatedly alternating between ambiguous paths.
- **params**
  - **gazebo_params.yaml**: Parameters for joint controllers in the Gazebo simulation.
  - **nav2_real_bot_params.yaml**: Parameters for configuring Navigation2 when running on the physical robot.
  - **nav2_rectangle_bot_params.yaml**: Parameters for configuring Navigation2 in simulation for the prototype rectangle bot.
  - **nav2_square_bot_params.yaml**: Parameters for configuring Navigation2 in simulation for the prototype square bot.
  - **rtabmap_params.yaml**: Parameters for configuring RTAB-Map.
- **rviz**
  - **robot_view.rviz**: Configuration file for RViz2 that defines what topics are visualized.

**lunabot_simulation**: This package contains assets and code for simulating the robot in Gazebo.
- **meshes**: Contains the 3D models used to visualize the robot in Gazebo and RViz2.
- **models**: Contains environmental models for the Gazebo simulation.
- **src**
  - **teleop**: Contains teleop scripts.
    - **keyboard_teleop.py**: Script for teleoping the robot using keyboard inputs.
  - **utils**: Contains utility nodes.
    - **blade_joint_controller.cpp**: Source code for controlling the bulldozer blade's joint.
    - **topic_remapper.cpp**: Remaps Gazebo controller topic names.
- **urdf**: Contains URDF descriptions.
  - **robot**
    - **real**
      - **trencher_bot.xacro**: URDF description of a high resolution real robot that utilizes a trencher digging mechanism.
    - simulation**
      - **realistic_rectangle_bot.xacro**: URDF description of a high resolution simulated rectangular bulldozer robot.
      - **rectangle_bot.xacro**: URDF description of a low resolution simulated rectangular bulldozer robot.
      - **square_bot.xacro**: URDF description of a low resolution simulated square bulldozer robot.
      - **trencher_bot.xacro**: URDF description of a high resolution simulated robot that utilizes a trencher digging mechanism.
- **worlds**: Gazebo world files for simulating the arena, each has different rock and crater placements.
  - **high_resolution**: Contains high resolution world files.
    - **artemis**: Contains world files simulating the Artemis arena.
      - **artemis_arena.world**
      - **artemis_arena2.world**
      - **artemis_arena3.world**
  - **low_resolution**: Contains low resolution world files.
    - **artemis**: Contains world files simulating the Artemis arena.
      - **artemis_arena.world**
      - **artemis_arena2.world**
      - **artemis_arena3.world**


**lunabot_system**: This package contains various autonomy/manual controllers and utilities.
- **action**
  - **Localization.action**: Action definition for localization.
- **src**
  - **control**
    - **localization_server.cpp**: Server responsible for handling localization with AprilTags.
    - **navigation_client.cpp**: Receives localization response and sends navigation goals and triggers robot behaviors when goals are reached.
    - **robot_controller.cpp**: Converts `/cmd_vel` commands and `/joy` inputs into physical motor speed outputs.
  - **utils**
    - **hardware_monitor.cpp**: Monitors hardware topics and outputs error messages if sensor data is not received.
    - **imu_rotator.cpp**: Processes and rotates IMU data into the East-North-Up (ENU) frame.
    - 
**lunabot_third_party** This folder contains third party packages.

**scripts**: This folder contains various setup and utility scripts.
- **config**
  - **99-realsense-d4xx-mipi-dfu.rules**: Udev rules for RealSense D456 cameras.
  - **99-realsense-libusb.rules**: Udev rules for RealSense cameras using the USB interface.
- **canable_start.sh**: Sets up the CAN bus interface for motor controller communication.
- **install_dependencies.sh**: Script to install required dependencies for the robot software.
- **setup_udev_rules.sh**: Script to set up udev rules for the Intel RealSense camera.

