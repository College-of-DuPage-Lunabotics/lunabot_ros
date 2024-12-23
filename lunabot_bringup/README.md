# lunabot_bringup

This package contains launch files to bring up autonomy nodes, Gazebo simulation, and real-world hardware.

## Launch Files
- **real_launch.py**: Launches the required nodes for bringing up the physical robot hardware and sensors, along with manual control and/or autonomy nodes.
- **sim_launch.py**: Launches the required nodes for simulating robot autonomy in Gazebo.
- **viz_launch.py**: Launches RViz2/Foxglove bridge and Gazebo to visualize the robot and its sensor data.

## Parameters

### real_launch.py
- `robot_mode`: Selects the mode of operation.
  - Options:
    - `manual`: Runs the robot in manual mode. **(Default)**
    - `auto`: Runs the robot in autonomous mode.
  - Example: `robot_mode:=auto`

### sim_launch.py
- `robot_type`: Defines which robot model parameters to use for Nav2.
  - Options:
    - `bulldozer`: Defines parameters for the bulldozer robot. **(Default)**
    - `trencher`: Defines parameters for the trencher robot.
  - Example: `robot_type:=trencher`

- `robot_mode`: Selects the mode of operation.
  - Options:
    - `manual`: Runs the robot in manual mode. **(Default)**
    - `auto`: Runs the robot in autonomous mode.
  - Example: `robot_mode:=auto`

- `teleop_mode`: Chooses the teleoperation method.
  - Options:
    - `keyboard`: Uses keyboard for teleoperation. **(Default)**
    - `xbox`: Uses Xbox controller for teleoperation.
  - Example: `teleop_mode:=xbox`

### viz_launch.py
- `robot_type`: Specifies the robot model to visualize.
  - Options:
    - `bulldozer`: Visualizes the bulldozer robot. **(Default)**
    - `trencher`: Visualizes the trencher robot.
  - Example: `robot_type:=trencher`

- `robot_heading`: Sets the initial orientation of the robot in Gazebo.
  - Options:
    - `north`: Points the robot north.
    - `south`: Points the robot south.
    - `east`: Points the robot east. **(Default)**
    - `west`: Points the robot west.
    - `random`: Assigns a random orientation.
  - Example: `robot_heading:=random`

- `use_sim`: Specifies whether to launch Gazebo or not.
  - Options:
    - `true`: Launches simulated robot in Gazebo. **(Default)**
    - `false`: Only launches RViz2 or Foxglove Studio instead of Gazebo, will receive data from real hardware.
  - Example: `use_sim:=false`

- `sim_gui`: Enables or disables the Gazebo GUI.
  - Options:
    - `true`: Runs Gazebo with its GUI. **(Default)**
    - `false`: Runs Gazebo in headless mode.
  - Example: `sim_gui:=false`

- `viz_type`: Choose between RViz2 or Foxglove Studio for visualization.
  - Options:
    - `rviz`: Opens visualization in RViz2. **(Default)**
    - `foxglove`: Launches Foxglove bridge to allow for connecting in the Foxglove Studio app.
  - Example: `viz_type:=foxglove`
