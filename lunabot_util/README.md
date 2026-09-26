# lunabot_util

This package contains various utility nodes.

## Source Files
- **bandwidth_monitor.py**: Monitors ROS topic bandwidth and publishes statistics.
- **image_compressor.py**: Compresses camera images for efficient network transport.
- **power_monitor.py**: Monitors power consumption and publishes power data.
- **topic_remapper.cpp**: Remaps Gazebo controller topic names.
- **livox_reorient.cpp**: Rotates the Livox lidar and IMU data into the base_link orientation using the URDF mount transform and drops points inside the robot body box.
