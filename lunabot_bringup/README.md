# lunabot_bringup

This package contains launch files to bring up the robot GUI, autonomy nodes, Gazebo simulation, and hardware.

There are three worlds available for Gazebo simulation and they come in both high and low resolution versions. They have different rock and crater placements and some are easier to navigate through than others. Modify lines 41 or 42 as shown below in `gui_launch.py` and rebuild this package if you'd like to change the world. The default world type is **artemis**. You can find the worlds listed [here](../lunabot_sim/README.md).

**gui_launch.py**
```python
world_files = {
  "ucf": os.path.join(sim_dir, "worlds", "high_resolution", "ucf", "ucf_arena.world"),
  "artemis": os.path.join(sim_dir, "worlds", "high_resolution", "artemis", "artemis_arena.world")
}
```

## Launch Files

### Main Launch File
- **gui_launch.py**: Primary launch file for both simulation and real robot. Launches visualization (RViz2 or custom PyQt GUI), robot state publisher, and handles both simulation (Gazebo) and real robot modes. Automatically starts joy_node in real mode for controller input.

### Component Launch Files
These are launched individually via GUI buttons or for specific subsystems:
- **hardware_launch.py**: Launches hardware drivers, cameras, LiDAR, and controller input for real robot
- **actions_launch.py**: Launches action servers for excavation, dumping, and homing operations
- **pointlio_launch.py**: Launches Point-LIO SLAM system
- **mapping_launch.py**: Launches RTAB-Map for dense 3D mapping
- **nav2_stack_launch.py**: Launches Nav2 navigation stack
- **localization_launch.py**: Launches AprilTag-based localization

## Parameters

### gui_launch.py

- `use_sim`: Specifies whether to run in simulation or real robot mode.
  - Options:
    - `true`: Launches simulated robot in Gazebo. **(Default)**
    - `false`: Real robot mode - uses real hardware data and automatically starts joy_node for controller input.
  - Example: `use_sim:=false`

- `viz_mode`: Selects the visualization interface.
  - Options:
    - `gui`: Launches custom PyQt5 GUI with camera feeds and telemetry. **(Default)**
    - `rviz`: Launches traditional RViz2 3D visualization.
  - Example: `viz_mode:=rviz`

- `robot_heading`: Sets the initial orientation of the robot in Gazebo (simulation only).
  - Options:
    - `north`: Points the robot north. **(Default)**
    - `south`: Points the robot south.
    - `east`: Points the robot east.
    - `west`: Points the robot west.
  - Example: `robot_heading:=south`

- `arena_type`: Selects the arena world file (simulation only).
  - Options:
    - `artemis`: Selects the Artemis arena world file. **(Default)**
    - `ucf`: Selects the UCF arena world file.
  - Example: `arena_type:=ucf`

- `sim_gui`: Enables or disables the Gazebo GUI (simulation only).
  - Options:
    - `true`: Runs Gazebo with its GUI. **(Default)**
    - `false`: Runs Gazebo in headless mode.
  - Example: `sim_gui:=false`

### Remote Robot Parameters (for gui_launch.py with custom GUI)

When using the custom PyQt GUI, you can configure remote robot operation:

- `robot_host`: Robot PC hostname or IP address for SSH commands.
  - Default: `localhost`
  - Example: `robot_host:=192.168.1.100`

- `robot_user`: SSH username for robot PC.
  - Default: Current user (`$USER`)
  - Example: `robot_user:=lunabot`

- `robot_workspace`: Workspace path on robot PC.
  - Default: `~/lunabot_ws`
  - Example: `robot_workspace:=/home/robot/lunabot_ws`

## Usage Examples

### Simulation
```bash
# Default - GUI with Gazebo
ros2 launch lunabot_bringup gui_launch.py

# RViz with Gazebo
ros2 launch lunabot_bringup gui_launch.py viz_mode:=rviz

# Different arena
ros2 launch lunabot_bringup gui_launch.py arena_type:=ucf

# Headless Gazebo
ros2 launch lunabot_bringup gui_launch.py sim_gui:=false
```

### Real Robot - Local
```bash
# GUI on robot PC (joy_node starts automatically)
ros2 launch lunabot_bringup gui_launch.py use_sim:=false

# RViz on robot PC
ros2 launch lunabot_bringup gui_launch.py use_sim:=false viz_mode:=rviz
```

### Real Robot - Remote
```bash
# GUI on laptop, hardware controlled on robot PC via SSH
ros2 launch lunabot_bringup gui_launch.py \
  use_sim:=false \
  robot_host:=192.168.1.100 \
  robot_user:=lunabot \
  robot_workspace:=~/lunabot_ws
```

