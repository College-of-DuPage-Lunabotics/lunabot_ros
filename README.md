# Project Overview

This repository contains the software developed by the College of DuPage team for the 2024-2025 NASA Lunabotics competition. It is built for ROS 2 Humble on Ubuntu 22.04. 

**This branch is for the Yahboom ROSMASTER X3 robot but with custom different code for mapping and navigation.**

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
git clone -b yahboom https://github.com/grayson-arendt/Lunabotics-2025.git
```

#### 3. Install dependencies

Run the installation script to install the required dependencies and setup the udev rules for the Astra camera.. `chmod +x` gives permission for the script to be executable.

```bash
cd ~/lunabot_ws/src/Lunabotics-2025/scripts
chmod +x install.sh setup_udev.sh
sudo ./setup_udev.sh
./install.sh
```

#### 4. Build the workspace

```bash
cd ~/lunabot_ws
colcon build --symlink-install
```

The flag `--symlink-install` will automatically reflect the changes in `Python, URDF, Xacro, and YAML` files, you will only need to rebuild if you change something other than those types.

## Running the robot

#### 1. Navigate to workspace and source setup

```bash
cd ~/lunabot_ws
source install/setup.bash
```
#### 2. Visualize with RViz2 and run robot

```bash
ros2 launch lunabot_bringup robot_launch.py
```