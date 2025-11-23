import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    GroupAction,
)

def generate_launch_description():
    config_dir = get_package_share_directory("lunabot_config")
    realsense_dir = get_package_share_directory("realsense2_camera")
    livox_dir = get_package_share_directory("livox_ros_driver2")

    apriltag_params_file = os.path.join(
        config_dir, "params", "apriltag", "tag_params.yaml"
    )
    bt_nav_to_pose = os.path.join(
        config_dir, "behavior_trees", "nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml"
    )
    bt_nav_through_poses = os.path.join(
        config_dir, "behavior_trees", "nav_through_poses_w_replanning_and_recovery.xml"
    )
    nav2_params_file = os.path.join(
        config_dir, "params", "nav2", "nav2_real_bot_params.yaml"
    )
    rtabmap_params_file = os.path.join(
        config_dir, "params", "rtabmap", "rtabmap_params.yaml"
    )

    livox_params_file = os.path.join(
        config_dir, "params", "mid360", "mid360.json"
    )
    
    declare_robot_mode = DeclareLaunchArgument(
        "robot_mode", default_value="manual", choices=["manual", "auto"]
    )

    slam_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="log",
        parameters=[
            {
                "use_sim_time": False,
                "subscribe_depth": True,
                "subscribe_rgbd": False,
                "subscribe_rgb": True,
                "subscribe_odom_info": False,
                "odom_sensor_sync": True,
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "odom",
                "publish_tf": True,
                "publish_tf_odom": False,
                "database_path": "",
                "approx_sync": True,
                "queue_size": 30,
                "approx_sync_max_interval": 0.1,
                "subscribe_scan_cloud": True,
                "subscribe_scan": False,
                "wait_imu_to_init": True,
            },
            rtabmap_params_file,
        ],
        remappings=[
            ("rgb/image", "/d456/color/image_raw"),
            ("depth/image", "/d456/depth/image_rect_raw"),
            ("rgb/camera_info", "/d456/color/camera_info"),
            ("scan_cloud", "/livox/lidar"),
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    excavation_server_node = Node(
        package="lunabot_nav",
        executable="excavation_server",
        name="excavation_server",
        output="screen",
    )

    localization_server_node = Node(
        package="lunabot_nav",
        executable="localization_server",
        name="localization_server",
        output="screen",
    )

    navigation_client_node = Node(
        package="lunabot_nav",
        executable="navigation_client",
        name="navigation_client",
        output="screen",
    )

    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params_file],
    )

    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params_file],
    )

    behavior_server_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params_file],
    )

    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            nav2_params_file,
            {"default_nav_to_pose_bt_xml": bt_nav_to_pose, "default_nav_through_poses_bt_xml": bt_nav_through_poses},
        ],
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{"autostart": True}, {"node_names": [
            "controller_server",
            "planner_server",
            "behavior_server",
            "bt_navigator",
        ]}, {"node_timeout": 10.0}],
    )

    d456_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "camera_name": "d456",
            "camera_namespace": "",
            "device_type": "d456",
            "publish_tf": "true",
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "2",
            "depth_module.depth_profile": "640x480x60",
            "rgb_camera.color_profile": "640x480x60",
        }.items(),
    )

    

    d456_imu_filter = Node(
        package="imu_complementary_filter",
        executable="complementary_filter_node",
        name="complementary_filter_gain_node",
        output="screen",
        parameters=[
            {"publish_tf": False},
            {"fixed_frame": "odom"},
            {"do_bias_estimation": True},
            {"do_adaptive_gain": True},
            {"use_mag": False},
            {"gain_acc": 0.01},
            {"gain_mag": 0.01},
        ],
        remappings=[
            ("imu/data_raw", "/d456/imu/data_raw"),
            ("imu/data", "/d456/imu/data"),
        ],
    )

    controller_teleop_node = Node(
        package="lunabot_teleop",
        executable="controller_teleop",
        name="controller_teleop",
        arguments=["--ros-args", "--log-level", "info"],
    )

    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    apriltag_d456_node = Node(
        package='apriltag_ros', executable='apriltag_node', output='screen',
        parameters=[apriltag_params_file],
        remappings=[('/image_rect', '/d456/color/image_raw'),
                    ('/camera_info', '/d456/color/camera_info')])

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            {"xfer_format": 0},  # 0 = PointCloud2 (PointXYZRTL), 1 = CustomMsg
            {"multi_topic": 0},  # 0 = single /livox/lidar topic
            {"data_src": 0},     # 0 = lidar data source
            {"publish_freq": 10.0},
            {"output_data_type": 0},
            {"frame_id": "livox_frame"},
            {"user_config_path": livox_config_path},
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_mode)
    ld.add_action(livox_driver)
    ld.add_action(d456_launch)
    ld.add_action(d456_imu_filter)
    ld.add_action(apriltag_d456_node)
    ld.add_action(map_to_odom_tf)
    ld.add_action(controller_teleop_node)

    ld.add_action(
        GroupAction(
            actions=[
                TimerAction(
                    period=2.0,
                    actions=[
                    ],
                ),
                TimerAction(
                    period=8.0,
                    actions=[
                        slam_node,
                    ],
                ),
                TimerAction(
                    period=15.0,
                    actions=[
                        controller_server_node,
                        planner_server_node,
                        behavior_server_node,
                        bt_navigator_node,
                        lifecycle_manager_node,
                    ],
                ),
            ],
            condition=LaunchConfigurationEquals("robot_mode", "manual"),
        )
    )

    ld.add_action(
        GroupAction(
            actions=[
                TimerAction(
                    period=3.0,
                    actions=[
                        excavation_server_node,
                        #localization_server_node,
                        navigation_client_node,
                    ],
                ),
                TimerAction(
                    period=5.0,
                    actions=[
                        slam_node,
                    ],
                ),
                TimerAction(
                    period=7.0,
                    actions=[
                        controller_server_node,
                        planner_server_node,
                        behavior_server_node,
                        bt_navigator_node,
                        lifecycle_manager_node,
                    ],
                ),
            ],
            condition=LaunchConfigurationEquals("robot_mode", "auto"),
        )
    )

    return ld
