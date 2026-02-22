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

### Point-LIO
- **Description**: Point-LIO LiDAR-inertial odometry, publishes to `/lio_odom`
- **Branch**: `grid_map_ros2` (commit `155f46d`)
- **Original source**: https://github.com/LihanChen2004/Point-LIO
- **Notes**: Modified for ROS2 Humble (`const SharedPtr` callbacks, C++17), added configurable `odom_frame_id`/`base_frame_id` params, TF publishing disabled, using UKF from robot_localization instead

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