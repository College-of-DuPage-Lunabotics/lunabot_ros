# Project Overview

This repository contains the software developed by the College of DuPage team for the 2024-2025 NASA Lunabotics competition. It is built for ROS 2 Humble on Ubuntu 22.04. 

**This branch is for simulating the robot only and only supports one camera. It is meant to run on computers with limited resources.**

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
git clone -b lightweight https://github.com/grayson-arendt/Lunabotics-2025.git
```

#### 3. Install dependencies

Run the installation script to install the required dependencies. `chmod +x` gives permission for the script to be executable.

```bash
cd ~/lunabot_ws/src/Lunabotics-2025/scripts
chmod +x install_dependencies.sh
./install_dependencies.sh
```

#### 4. Build the workspace

```bash
cd ~/lunabot_ws
colcon build --symlink-install
```

The flag `--symlink-install` will automatically reflect the changes in `Python, URDF, Xacro, and YAML` files, you will only need to rebuild if you change something other than those types.


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


## Project Structure

**lunabot_bringup**: This package contains launch files to bring up autonomy nodes and Gazebo simulation.
- **launch**
  - **simulation_launch.py**: Launches the required nodes for simulating robot autonomy in Gazebo.
  - **visualization_launch.py**: Launches RViz2 and Gazebo to visualize the robot and its sensor data.

**lunabot_config**: This package contains configuration files for Navigation2 behavior trees, RViz2 settings, and various 
- **behavior_trees**
  - **navigate_through_poses_w_replanning_and_recovery.xml**: A behavior tree used with Navigation2 to implement behaviors like goal replanning and recovery for NavigateThroughPoses action.
  - **nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml**: A behavior tree used with Navigation2 to only replan when path becomes invalid to prevent Navigation2 from repeatedly alternating between ambiguous paths.
- **params**
  - **ekf_params.yaml**: Parameters for robot_localization Extended Kalman Filter (EKF) node.
  - **gazebo_bulldozer_bot_params.yaml**: Parameters for bulldozer style robot joint controllers in the Gazebo simulation.
  - **gazebo_trencher_bot_params.yaml**: Parameters for trencher style robot joint controllers in the Gazebo simulation.
  - **nav2_rectangle_bot_params.yaml**: Parameters for configuring Navigation2 in simulation for the rectangular bulldozer bot.
  - **nav2_square_bot_params.yaml**: Parameters for configuring Navigation2 in simulation for the square bulldozer bot.
  - **nav2_trencher_bot_params.yaml**: Parameters for configuring Navigation2 in simulation for the trencher robot.
  - **rtabmap_params.yaml**: Parameters for configuring RTAB-Map.
- **rviz**
  - **robot_view.rviz**: Configuration file for RViz2 that defines what topics are visualized.

**lunabot_simulation**: This package contains assets and code for simulating the robot in Gazebo.
- **models**: Contains environmental models for the Gazebo simulation.
- **src**
  - **teleop**: Contains teleop scripts.
    - **keyboard_teleop.py**: Script for teleoping the robot using keyboard inputs.
  - **utils**: Contains utility nodes.
    - **blade_joint_controller.cpp**: Source code for controlling the bulldozer blade's joint.
    - **topic_remapper.cpp**: Remaps Gazebo controller topic names.
- **urdf**: Contains URDF descriptions.
  - **robot**
      - **rectangle_bot.xacro**: URDF description of a low resolution simulated rectangular bulldozer robot.
      - **square_bot.xacro**: URDF description of a low resolution simulated square bulldozer robot.
      - **trencher_bot.xacro**: URDF description of a high resolution simulated robot that utilizes a trencher digging mechanism.
  - **worlds**: Gazebo world files for simulating the arena, each has different rock and crater placements.
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


**scripts**: This folder contains setup scripts.
- **install_dependencies.sh**: Script to install required dependencies.

