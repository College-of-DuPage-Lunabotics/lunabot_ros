# Third Party Packages

This directory contains third-party ROS2 packages that have been integrated directly into the repository (not as submodules).

## Packages

### livox_ros_driver2
- **Description**: ROS2 driver for Livox LiDAR sensors
- **Modified for**: ROS2 Humble (stripped ROS1 support, uses `launch/` directory)
- **Original source**: https://github.com/Livox-SDK/livox_ros_driver2

### livox_laser_simulation_ros2
- **Description**: Gazebo simulation plugin for Livox LiDAR
- **Original source**: https://github.com/Livox-SDK/livox_laser_simulation

### kiss-icp
- **Description**: KISS-ICP (Keep It Simple, Stupid - Iterative Closest Point) for LiDAR odometry
- **Original source**: https://github.com/PRBonn/kiss-icp

## Why Not Submodules?

These packages are maintained as regular directories (not git submodules) because:
1. We've made custom modifications for our specific ROS2 Humble setup
2. Easier for team members to clone and build without submodule complexity
3. Changes are tracked directly in our main repository
