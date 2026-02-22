import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart, OnProcessExit


def generate_launch_description():
    config_dir = get_package_share_directory("lunabot_config")
    realsense_dir = get_package_share_directory("realsense2_camera")

    apriltag_params_file = os.path.join(
        config_dir, "params", "apriltag", "tag_params.yaml"
    )
    bt_nav_to_pose = os.path.join(
        config_dir,
        "behavior_trees",
        "nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml",
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

    livox_params_file = os.path.join(config_dir, "params", "mid360", "mid360.json")

    point_lio_config = os.path.join(
        config_dir,
        "params",
        "point_lio",
        "mid360.yaml",
    )

    declare_robot_mode = DeclareLaunchArgument(
        "robot_mode", default_value="manual", choices=["manual", "auto"]
    )

    declare_use_localization = DeclareLaunchArgument(
        "use_localization",
        default_value="true",
        choices=["true", "false"],
        description="Enable AprilTag localization phase before navigation (default: true).",
    )

    rgbd_sync_front = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync_front",
        output="screen",
        parameters=[
            {"use_sim_time": False, "approx_sync": True, "sync_queue_size": 1000}
        ],
        remappings=[
            ("rgb/image", "/d456_front/color/image_raw"),
            ("depth/image", "/d456_front/depth/image_rect_raw"),
            ("rgb/camera_info", "/d456_front/color/camera_info"),
            ("rgbd_image", "/d456_front/rgbd_image"),
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    rgbd_sync_back = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync_back",
        output="screen",
        parameters=[
            {"use_sim_time": False, "approx_sync": True, "sync_queue_size": 1000}
        ],
        remappings=[
            ("rgb/image", "/d456_back/color/image_raw"),
            ("depth/image", "/d456_back/depth/image_rect_raw"),
            ("rgb/camera_info", "/d456_back/color/camera_info"),
            ("rgbd_image", "/d456_back/rgbd_image"),
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    slam_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="log",
        parameters=[
            {
                "use_sim_time": False,
                "subscribe_depth": False,
                "subscribe_rgbd": True,
                "rgbd_cameras": 2,
                "subscribe_rgb": False,
                "subscribe_odom_info": False,
                "odom_sensor_sync": False,
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "odom",
                "publish_tf": True,
                "publish_tf_odom": False,
                "database_path": "",
                "approx_sync": True,
                "sync_queue_size": 1000,
                "wait_for_transform": 0.2,
                "subscribe_scan_cloud": True,
                "subscribe_scan": False,
                "wait_imu_to_init": False,
                "subscribe_odom": True,
            },
            rtabmap_params_file,
        ],
        remappings=[
            ("rgbd_image0", "/d456_front/rgbd_image"),
            ("rgbd_image1", "/d456_back/rgbd_image"),
            ("scan_cloud", "/livox/lidar_PointCloud2"),
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="laserMapping",
        output="screen",
        parameters=[
            point_lio_config,
            {
                "use_sim_time": False,
                "use_imu_as_input": False,
                "prop_at_freq_of_imu": True,
                "check_satu": False,
                "init_map_size": 10,
                "point_filter_num": 3,
                "space_down_sample": True,
                "filter_size_surf": 0.5,
                "filter_size_map": 0.5,
                "ivox_nearby_type": 26,
                "runtime_pos_log_enable": False,
                "odom_header_frame_id": "odom",
                "odom_child_frame_id": "base_link",
            },
        ],
        remappings=[
            ("/Odometry", "/odom"),
        ],
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
        condition=LaunchConfigurationEquals("use_localization", "true"),
    )

    navigation_client_node = Node(
        package="lunabot_nav",
        executable="navigation_client",
        name="navigation_client",
        output="screen",
        parameters=[{"use_localization": LaunchConfiguration("use_localization")}],
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
            {
                "default_nav_to_pose_bt_xml": bt_nav_to_pose,
                "default_nav_through_poses_bt_xml": bt_nav_through_poses,
            },
        ],
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"autostart": True},
            {
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                ]
            },
            {"node_timeout": 10.0},
        ],
    )

    d456_front_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "camera_name": "d456_front",
            "camera_namespace": "",
            "device_type": "d456",
            # "serial_no": "", # add once camera is setup
            "publish_tf": "true",
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "2",
            "depth_module.depth_profile": "640x480x60",
            "rgb_camera.color_profile": "640x480x60",
        }.items(),
    )

    d456_back_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "camera_name": "d456_back",
            "camera_namespace": "",
            "device_type": "d456",
            # "serial_no": "", # add once camera is setup
            "publish_tf": "true",
            "enable_gyro": "false",
            "enable_accel": "false",
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
            ("imu/data_raw", "/d456_front/imu/data_raw"),
            ("imu/data", "/d456_front/imu/data"),
        ],
    )

    controller_teleop_node = Node(
        package="lunabot_teleop",
        executable="controller_teleop",
        name="controller_teleop",
        arguments=["--ros-args", "--log-level", "info"],
    )

    apriltag_d456_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        output="screen",
        parameters=[apriltag_params_file],
        remappings=[
            ("/image_rect", "/d456_front/color/image_raw"),
            ("/camera_info", "/d456_front/color/camera_info"),
        ],
    )

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            {"xfer_format": 1},
            {"multi_topic": 0},
            {"data_src": 0},
            {"publish_freq": 10.0},
            {"output_data_type": 1},
            {"frame_id": "mid360_lidar_link"},
            {"user_config_path": livox_params_file},
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_mode)
    ld.add_action(declare_use_localization)
    ld.add_action(livox_driver)
    ld.add_action(d456_front_launch)
    ld.add_action(d456_back_launch)
    ld.add_action(d456_imu_filter)
    ld.add_action(controller_teleop_node)
    ld.add_action(rgbd_sync_front)
    ld.add_action(rgbd_sync_back)
    ld.add_action(apriltag_d456_node)
    ld.add_action(localization_server_node)

    manual_odometry_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=rgbd_sync_front,
            on_start=[point_lio_node],
        ),
        condition=LaunchConfigurationEquals("robot_mode", "manual"),
    )

    manual_slam_after_localization_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=localization_server_node,
            on_exit=[slam_node],
        ),
        condition=LaunchConfigurationEquals("use_localization", "true"),
    )

    manual_slam_without_loc_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=point_lio_node,
            on_start=[slam_node],
        ),
        condition=LaunchConfigurationEquals("use_localization", "false"),
    )

    manual_slam_group = GroupAction(
        actions=[
            manual_slam_after_localization_handler,
            manual_slam_without_loc_handler,
        ],
        condition=LaunchConfigurationEquals("robot_mode", "manual"),
    )

    manual_nav2_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=slam_node,
            on_start=[
                controller_server_node,
                planner_server_node,
                behavior_server_node,
                bt_navigator_node,
                lifecycle_manager_node,
            ],
        ),
        condition=LaunchConfigurationEquals("robot_mode", "manual"),
    )

    manual_teleop = GroupAction(
        actions=[controller_teleop_node],
        condition=LaunchConfigurationEquals("robot_mode", "manual"),
    )

    ld.add_action(manual_odometry_handler)
    ld.add_action(manual_slam_group)
    ld.add_action(manual_nav2_handler)
    ld.add_action(manual_teleop)

    auto_nav2_stack = GroupAction(
        actions=[
            controller_server_node,
            planner_server_node,
            behavior_server_node,
            bt_navigator_node,
            lifecycle_manager_node,
        ],
        condition=LaunchConfigurationEquals("robot_mode", "auto"),
    )

    auto_odometry_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_manager_node,
            on_start=[point_lio_node],
        ),
        condition=LaunchConfigurationEquals("robot_mode", "auto"),
    )

    slam_after_localization_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=localization_server_node,
            on_exit=[slam_node],
        ),
        condition=LaunchConfigurationEquals("use_localization", "true"),
    )

    auto_slam_without_loc_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=point_lio_node,
            on_start=[slam_node],
        ),
        condition=LaunchConfigurationEquals("use_localization", "false"),
    )

    auto_slam_group = GroupAction(
        actions=[
            slam_after_localization_handler,
            auto_slam_without_loc_handler,
        ],
        condition=LaunchConfigurationEquals("robot_mode", "auto"),
    )

    auto_clients_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=slam_node,
            on_start=[excavation_server_node, navigation_client_node],
        ),
        condition=LaunchConfigurationEquals("robot_mode", "auto"),
    )

    ld.add_action(auto_nav2_stack)
    ld.add_action(auto_odometry_handler)
    ld.add_action(auto_slam_group)
    ld.add_action(auto_clients_handler)

    return ld
