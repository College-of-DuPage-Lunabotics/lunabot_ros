# lunabot_config

This package contains configuration files for Nav2 behavior trees, RViz2 configurations, and various parameters.

## Behavior Trees
- **nav_through_poses_w_replanning_and_recovery.xml**: Implements behaviors like goal replanning and recovery for NavigateThroughPoses action.
- **nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml**: Replans only when the path becomes invalid to prevent Nav2 from repeatedly alternating between ambiguous paths.

## Parameters
- **apriltag**
  - **tag_params.yaml**: AprilTag detection parameters.
- **gazebo**
  - **gazebo_v1_bot_params.yaml**: V1 robot joint controller parameters for Gazebo.
  - **gazebo_v2_bot_params.yaml**: V2 robot joint controller parameters for Gazebo.
- **mid360**
  - **mid360.json**: Livox Mid-360 LiDAR driver configuration.
- **nav2**
  - **nav2_v1_bot_params.yaml**: Nav2 parameters for simulation.
  - **nav2_real_bot_params.yaml**: Nav2 parameters for physical robot.
- **point_lio**
  - **mid360.yaml**: Point-LIO configuration for Mid-360 LiDAR.
- **robot_localization**
  - **ukf_params.yaml**: Unscented Kalman Filter (UKF) parameters.
- **rtabmap**
  - **rtabmap_params.yaml**: RTAB-Map configuration parameters.

## RViz Configurations
- **robot_view.rviz**: Defines layout for visualization in RViz2.
