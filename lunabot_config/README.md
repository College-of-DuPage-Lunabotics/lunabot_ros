# lunabot_config

This package contains configuration files for Nav2 behavior trees, RViz2 configurations, and various parameters.

## Behavior Trees
- **nav_through_poses_w_replanning_and_recovery.xml**: Implements behaviors like goal replanning and recovery for NavigateThroughPoses action.
- **nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml**: Replans only when the path becomes invalid to prevent Nav2 from repeatedly alternating between ambiguous paths.

## Parameters
- **gazebo**
  - **gazebo_bulldozer_bot_params.yaml**: Bulldozer robot joint controller parameters for Gazebo.
  - **gazebo_trencher_bot_params.yaml**: Trencher robot joint controller parameters for Gazebo.
- **laser_filters**
  - **s2l_params.yaml**: S2L lidar scan filtering parameters.
  - **s3_params.yaml**: S3 lidar scan filtering parameters.
- **nav2**
  - **nav2_bulldozer_bot_params.yaml**: Nav2 parameters for bulldozer robot in simulation.
  - **nav2_real_bot_params.yaml**: Nav2 parameters for physical robot.
  - **nav2_trencher_bot_params.yaml**: Nav2 parameters for trencher robot in simulation.
- **robot_localization**
  - **ukf_params.yaml**: Unscented Kalman Filter (UKF) parameters.
- **rtabmap**
  - **rtabmap_params.yaml**: RTAB-Map configuration parameters.

## RViz Configurations
- **robot_view.rviz**: Defines layout for visualization in RViz2.
