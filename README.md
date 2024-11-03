# Project Overview

**This branch is for the Yahboom ROSMASTER X3 robot. It is ran on a Jetson Xavier NX with Ubuntu 20.04 and ROS 2 Foxy.**

### SSH Into the Robot Computer

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

Using the username and IP address from the previous step, now you can connect to the robot computer. It may look something like this:

```bash
ssh jetson-main@192.168.10.1 # (General format: username@ip_address)
```
It will ask you if you are sure you want to connect, type `yes`. Then, confirm by typing in the robot computer's password. Additionally, there is a VSCode extension called `Remote Explorer` that I highly recommend you use for developing code on the robot computer. The guide can be found [here](https://code.visualstudio.com/docs/remote/ssh#_connect-to-a-remote-host).

## Installation

#### 1. Setup workspace and clone repository (robot computer)

```bash
mkdir -p ~/lunabot_ws/src
cd ~/lunabot_ws/src
git clone -b yahboom https://github.com/grayson-arendt/Lunabotics-2025.git
```

#### 2. Install dependencies (robot computer)

Run the installation script to install the required dependencies and setup the udev rules for the Astra camera. `chmod +x` gives permission for the script to be executable.

```bash
cd ~/lunabot_ws/src/Lunabotics-2025/scripts
chmod +x install.sh setup_udev.sh
sudo ./setup_udev.sh
./install.sh
```

#### 3. Build the workspace

```bash
cd ~/lunabot_ws
colcon build --symlink-install
```

The flag `--symlink-install` will automatically reflect the changes in `Python, URDF, Xacro, and YAML` files, you will only need to rebuild if you change something other than those types.

## Running the Robot

#### 1. Connect Bluetooth controller (robot computer)

To put the controller into pairing mode, hold the `PS` and `SHARE` buttons until the light begins flashing rapidly.

```bash
bluetoothctl
scan on
```
There will be various devices that pop up, but it may look something like this:
```bash
[NEW] Device XX:XX:XX:XX:XX:XX Wireless Controller
```
Copy the Bluetooth MAC address after "Device" and run the following:

```bash
pair XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
exit
```

#### 2. Navigate to workspace and source setup (both robot and host computer)

```bash
cd ~/lunabot_ws
source install/setup.bash
```

#### 3. Visualize with RViz2 (host computer)

```bash
ros2 launch lunabot_bringup visualization_launch.py
```

#### 4. Run robot (robot computer)

```bash
ros2 launch lunabot_bringup robot_launch.py
```

Press the button that says `KEY` (located near the power cable on the board and the `ON` switch). If successful, the RGB lights will turn on and there will be a small beep noise. You will now be able to drive the robot using the left and right joysticks on the controller. The left joystick x-axis will spin the robot, however, as this is mecanum it does not work well and should only be used if very misaligned. The right joystick x-axis will move the robot from side to side, while the right joystick y-axis will move it forward and backwards.
