# Project Overview

This repository contains the software developed by the College of DuPage team for the NASA Lunabotics competition. It is built for ROS 2 Humble on Ubuntu 22.04 for x86-64 architecture.

## System Components

**Computer**
- ASRock 4X4 BOX-8840U

**Sensors**
- RPLidar S3
- RPLidar S2L
- Intel RealSense D455 Depth Camera
- Intel RealSense D456 Depth Camera

**Hardware**
- REV Robotics NEO Vortex (x2)
- REV Robotics Spark Max (x2)
- REV Robotics Power Distribution Hub
- Turnigy 14.8V 8000mAh LiPo Battery
- Turnigy 14.8V 12000mAh LiPo Battery
- ODrive USB-CAN Adapter

## Build Status

| ROS 2 Distro | Project Status |
|--------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Humble | [![build](https://github.com/College-of-DuPage-Lunabotics/lunabot_ros/actions/workflows/build.yml/badge.svg?branch=development)](https://github.com/College-of-DuPage-Lunabotics/lunabot_ros/actions/workflows/build.yml) |


## Installation

**Note: You will need to have already installed ROS 2 Humble before continuing with installation. The guide can be found [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html). Install both `ros-humble-desktop` and `ros-dev-tools`.**

#### (Optional) 1. Append lines to .bashrc

.bashrc is a script that runs everytime a new terminal window is opened and has various configurations, environment variables, and commands for setup. There is a bug in the VSCode terminal that will cause a symbol lookup error, so you have to unset the path variable using `unset GTK_path`. If you haven't already added `source /opt/ros/humble/setup.bash` to your .bashrc file, it simply runs the setup script for ROS 2 Humble.

```bash
echo 'unset GTK_PATH' >> ~/.bashrc
echo 'source /opt/ros/humble/setup.bash ' >> ~/.bashrc
```

This will permanently append these two lines to your .bashrc file, so there is no need to run it again. If you want to edit the file manually, use `nano ~/.bashrc` or `gedit ~/.bashrc` if you prefer a text editor GUI instead.

#### 2. Setup workspace and clone repository

```bash
mkdir -p ~/lunabot_ws/src
cd ~/lunabot_ws/src
git clone https://github.com/College-of-DuPage-Lunabotics/lunabot_ros.git
cd lunabot_ros
git submodule update --init --recursive --remote
```
**If you have previously cloned this repository and do not see anything in the folders located in `third_party_packages` after running `git pull`, run `git submodule update --init --recursive --remote` inside the `lunabot_ros` folder to initialize the submodules.**

#### 3. Install dependencies

Run the installation script to install the required dependencies. `chmod +x` gives permission for the script to be executable.

```bash
cd ~/lunabot_ws/src/lunabot_ros/scripts
chmod +x install_dependencies.sh
./install_dependencies.sh
```

#### (Optional) 4. Install Foxglove Studio

If you would prefer to use Foxglove Studio instead of RViz2 to visualize the robot, you can install it with:
```bash
sudo snap install foxglove-studio
```

#### 5. Build the workspace

Building may take some time due to the external packages in `third_party_packages`. Various flags such as `-DRTABMAP_SYNC_MULTI_RGBD=ON` need to be set to enable extra features for RTAB-Map.

To avoid building the entire workspace all over again after the initial build if you make changes, use `colcon build --packages-select name_of_package` and choose the package that you made changes to for rebuilding. You can list multiple packages after the `--packages-select` flag. You only need to rebuild the workspace if you modify a file for a compiled language such as `C++` or add new files, the flag `--symlink-install` will automatically reflect the changes in `Python, URDF, Xacro, and YAML` files.

```bash
cd ~/lunabot_ws
colcon build --symlink-install --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DWITH_OPENCV=ON -DWITH_APRILTAG=ON -DWITH_OPENGV=OFF --parallel-workers 1 # Modify number as needed
```

#### 6. (Optional) Set MAKEFLAG and Rebuild
If your computer keeps crashing while trying to build, `colcon build` may be trying to do too many things at once. Setting this flag to `-j1` limits each package's internal make jobs to 1 thread. You can either increase or reduce both this and `--parallel-workers`, increasing will make it build faster but may put more stress on your computer, leading to freezing.

```bash
export MAKEFLAGS="-j1" # Modify number as needed
```

Next, rebuild using the same commands in step **5. Build the workspace**.

## Simulating the Robot
The launch files have various parameters that can be set, such as changing the robot model, autonomy level, and choosing between RViz2 and Foxglove Studio for visualization. If you are using the parameter `vis_type:=foxglove`, refer to the [Foxglove guide](https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2/#foxglove-websocket) for connecting in the app. You can import the same layout I used by choosing `Import from file...` under the `LAYOUT` menu and selecting `foxglove_layout.json` from this directory.


**A detailed list of the launch parameters can be found [here](lunabot_bringup/README.md).**

#### 1. Navigate to workspace and source setup

```bash
cd ~/lunabot_ws
source install/setup.bash
```
#### Open separate terminal windows and source the workspace setup for each next step:

#### 2. Launch visualization

```bash
ros2 launch lunabot_bringup vis_launch.py 
```

#### 3. Launch mapping and navigation

```bash
ros2 launch lunabot_bringup sim_launch.py
```

##### RViz2
<p align="center">
  <img src="demo_rviz.png">
</p>

##### Foxglove Studio 
<p align="center">
  <img src="demo_foxglove.png">
</p>


## Running the Physical Robot

### SSH into Robot Computer

SSH (Secure Shell) allows you to access another device over the network and run commands. In this context:
- The **client** is your personal computer (e.g., your laptop).
- The **host** is the robot's onboard computer (e.g., ASRock 4X4 BOX-8840U).

#### 1. Install and enable SSH server (host):
```bash
sudo apt update
sudo apt install openssh-server

sudo systemctl start ssh
sudo systemctl enable ssh
```

#### 2. Install SSH client (client):

```bash
sudo apt update
sudo apt install openssh-client

```

#### 3. Create SSH-key (client)

```bash
ssh-keygen
```

#### 4. Get username and IP address (host)

```bash
whoami
```
This will return the username of the host, although you can also see the username just by looking at the terminal. It is the first name before the @, for example, the username would be `asrock` for `asrock@asrock-main`.

Next, get the IP address:
```bash
hostname -I
```

The IP address is the first set of numbers in the list.

#### 5. Establish SSH connection (client)

Using the username and IP address from the previous step, now you can connect to the host. It may look something like this for example:

```bash
ssh asrock@192.168.10.1 # (General format: username@ip_address)
```
 It will ask you if you are sure you want to connect, type `yes`. Then, confirm by typing in the host's password. 

### Configure Device Permissions 

#### 1. Add user to dialout group then restart (host)

```bash
sudo usermod -a -G dialout $USER
sudo reboot
```

Use `ls /dev/ttyUSB*` to identify device numbers if the lidars are disconnected and reconnected, then adjust the lidar `"serial_port"` parameters in `real_launch.py` accordingly.

#### 2. Setup camera udev rules (host)

```bash
cd ~/lunabot_ws/src/lunabot_ros/scripts
chmod +x setup_udev_rules.sh
sudo ./setup_udev_rules.sh
```

Make sure all cameras are unplugged while setting up the udev rules.

### Running Launch Files

#### 1. Source workspace setup (both client and host)

```bash
cd ~/lunabot_ws
source install/setup.bash
```

#### 2. Connect controller and run joy node (client)

Connect your controller either through a wired or Bluetooth connection to the client.

```bash
ros2 run joy joy_node
```

#### 3. Visualize with RViz2 (client)

```bash
ros2 launch lunabot_bringup vis_launch.py use_sim:=false
```

#### 4. Launch the real robot (host)

```bash
ros2 launch lunabot_bringup real_launch.py
```