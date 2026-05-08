# Lunabot GUI

PyQt5-based graphical interface for Lunabot robot control and monitoring.

## Launch Options

### Real Robot (Default)
```bash
# Launch GUI with real robot (default)
ros2 launch lunabot_bringup gui_launch.py

# Or run GUI directly
ros2 run lunabot_gui lunabot_gui.py -p mode:=false
```

### Simulation Mode
```bash
# Launch GUI with simulation
ros2 launch lunabot_bringup gui_launch.py use_sim:=true

# Or run GUI directly
ros2 run lunabot_gui lunabot_gui.py -p mode:=true
```

### Real Robot - Remote Operation (Recommended)

Run GUI on laptop, execute hardware/SLAM on robot PC via SSH.

```bash
# Run GUI on laptop with remote robot parameters
ros2 run lunabot_gui lunabot_gui.py \
  -p mode:=false \
  -p robot_host:=192.168.1.100 \
  -p robot_user:=lunabot \
  -p robot_workspace:=~/lunabot_ws

# Example with hostname instead of IP
ros2 run lunabot_gui lunabot_gui.py \
  -p mode:=false \
  -p robot_host:=lunabot-robot.local \
  -p robot_user:=pi \
  -p robot_workspace:=~/lunabot_ws
```

## Parameters

- `mode`: Specifies whether to run in simulation or real robot mode.
  - Options:
    - `true`: Simulation mode.
    - `false`: Real robot mode. **(Default)**
  - Example: `mode:=true`

- `robot_host`: Robot PC hostname or IP address for SSH commands (real mode only).
  - Default: `localhost`
  - Example: `robot_host:=192.168.1.100`

- `robot_user`: SSH username for robot PC (real mode only).
  - Default: Current user (`$USER`)
  - Example: `robot_user:=lunabot`

- `robot_workspace`: Workspace path on robot PC (real mode only).
  - Default: `~/lunabot_ws`
  - Example: `robot_workspace:=/home/robot/lunabot_ws`

## Remote Operation Setup

For laptop-based operation with remote robot PC:

### 1. Configure SSH key authentication (no password prompts)
```bash
# On laptop - generate SSH key
ssh-keygen -t ed25519

# Copy key to robot PC
ssh-copy-id your_user@robot_ip

# Test connection (should not ask for password)
ssh your_user@robot_ip 'echo success'
```

### 2. Ensure matching ROS_DOMAIN_ID on both machines
```bash
# Add to ~/.bashrc on both laptop and robot PC
export ROS_DOMAIN_ID=42  # Use any number 0-232
```

### 3. Launch GUI on laptop
```bash
# GUI on laptop subscribes to ROS topics from robot
ros2 run lunabot_gui lunabot_gui.py \
  -p mode:=false \
  -p robot_host:=<robot_ip> \
  -p robot_user:=<username>
```

## What Runs Where

**Robot PC (in real mode):**
- Hardware drivers (cameras, LiDAR, IMU)
- CAN interface
- Controller teleop
- Action servers (assisted excavation/depositing for teleop, auto excavation/depositing for autonomy)
- SLAM (Point-LIO)
- Mapping (RTAB-Map)
- Navigation (Nav2)

**Laptop (always):**
- GUI (visualization and control)
- RViz (optional)

**Communication:**
- All ROS topics flow over the network via DDS
- Both machines must have matching ROS_DOMAIN_ID

## Action Servers

The GUI interacts with the following action servers for manual operations:
- **assisted_excavation_action**: Teleop-assisted excavation with full vibration control
- **assisted_depositing_action**: Teleop-assisted depositing sequence

For autonomous operations, the navigation client uses:
- **auto_excavation_action**: Autonomous excavation with minimal vibration
- **auto_depositing_action**: Autonomous depositing with forward drive positioning
