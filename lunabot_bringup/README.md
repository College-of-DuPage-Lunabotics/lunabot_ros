# lunabot_bringup

This package contains launch files to bring up autonomy nodes, Gazebo simulation, and real-world hardware.

There are three worlds available for Gazebo simulation and they come in both high and low resolution versions. They have different rock and crater placements and some are easier to navigate through than others. Modify lines 41 or 42 as shown below in `viz_launch.py` and rebuild this package if you'd like to change the world. The default world type is **artemis**. You can find the worlds listed [here](../lunabot_sim/README.md).

**viz_launch.py**
```python
world_files = {
  "ucf": os.path.join(sim_dir, "worlds", "high_resolution", "ucf", "ucf_arena.world"),
  "artemis": os.path.join(sim_dir, "worlds", "high_resolution", "artemis", "artemis_arena.world")
}
```

## Launch Files
- **real_launch.py**: Launches the required nodes for bringing up the physical robot hardware and sensors, along with manual control and/or autonomy nodes.
- **sim_launch.py**: Launches the required nodes for simulating robot autonomy in Gazebo.
- **viz_launch.py**: Launches RViz2 and Gazebo to visualize the robot and its sensor data.

## Parameters

### real_launch.py
- `robot_mode`: Selects the mode of operation.
  - Options:
    - `manual`: Runs the robot in manual mode. **(Default)**
    - `auto`: Runs the robot in autonomous mode.
  - Example: `robot_mode:=auto`

- `use_localization`: Enables AprilTag localization before navigation.
  - Options:
    - `true`: Waits for successful AprilTag localization before starting SLAM. **(Default)**
    - `false`: Skips localization and starts SLAM immediately.
  - Example: `use_localization:=false`

### sim_launch.py
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

- `use_localization`: Enables AprilTag localization before navigation.
  - Options:
    - `true`: Waits for successful AprilTag localization before starting SLAM. **(Default)**
    - `false`: Skips localization and starts SLAM immediately.
  - Example: `use_localization:=false`

### viz_launch.py
- `robot_heading`: Sets the initial orientation of the robot in Gazebo.
  - Options:
    - `north`: Points the robot north. **(Default)**
    - `south`: Points the robot south.
    - `east`: Points the robot east.
    - `west`: Points the robot west.
  - Example: `robot_heading:=south`

- `arena_type` : Selects the arena world file.
  -  Options:
     -  `ucf`: Selects the UCF arena world file.
     -  `artemis`: Selects the Artemis arena world file.
  - Example: `arena_type:=artemis`

- `use_sim`: Specifies whether to launch Gazebo or not.
  - Options:
    - `true`: Launches simulated robot in Gazebo. **(Default)**
    - `false`: Only launches RViz2 instead of Gazebo, will receive data from real hardware.
  - Example: `use_sim:=false`

- `sim_gui`: Enables or disables the Gazebo GUI.
  - Options:
    - `true`: Runs Gazebo with its GUI. **(Default)**
    - `false`: Runs Gazebo in headless mode.
  - Example: `sim_gui:=false`

