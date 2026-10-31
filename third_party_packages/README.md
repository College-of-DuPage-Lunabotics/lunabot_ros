# Third Party Packages

This directory contains third-party ROS2 packages integrated directly into the repository.

## Packages

### livox_ros_driver2
- **Description**: ROS2 driver for Livox LiDAR sensors
- **Original source**: https://github.com/Livox-SDK/livox_ros_driver2
- **Notes**: Modified for ROS2 Humble (stripped ROS1 support, uses `launch/` directory)

### livox_laser_simulation_RO2
- **Description**: Gazebo Classic simulation plugin for Livox LiDAR sensors
- **Original source**: https://github.com/Livox-SDK/livox_laser_simulation
- **Notes**: Modified for CustomMsg and PointCloud2 publishing

### FAST-LIO
- **Description**: FAST-LIO2 LiDAR-inertial odometry, the LIO used by the robot (`fastlio_launch.py`)
- **Notes**: Added configurable `odom_frame_id`/`base_frame_id` params so the body cloud uses the TF frame published by `lio_odom_to_base`, TF publishing disabled. `lio_odom_to_base` shifts its odometry to base_link and publishes `/lio_odom`

### Point-LIO
- **Description**: Point-LIO LiDAR-inertial odometry, kept for comparison (`pointlio_launch.py`, not used by the GUI)
- **Branch**: `grid_map_ros2` (commit `155f46d`)
- **Original source**: https://github.com/LihanChen2004/Point-LIO
- **Notes**: Modified for ROS2 Humble (`const SharedPtr` callbacks, C++17), added configurable `odom_frame_id`/`base_frame_id` params, TF publishing disabled. `lio_odom_to_base` shifts its odometry to base_link and publishes `/lio_odom`

### rtabmap
- **Description**: RTAB-Map core library for real-time SLAM
- **Branch**: `0.21.9-humble`
- **Original source**: https://github.com/introlab/rtabmap
- **Notes**: Built with `-DRTABMAP_SYNC_MULTI_RGBD=ON` for dual-camera support

### rtabmap_ros
- **Description**: ROS2 wrapper for RTAB-Map
- **Branch**: `0.21.9-humble`
- **Original source**: https://github.com/introlab/rtabmap_ros
- **Notes**: Built with `-DRTABMAP_SYNC_MULTI_RGBD=ON` for dual-camera support