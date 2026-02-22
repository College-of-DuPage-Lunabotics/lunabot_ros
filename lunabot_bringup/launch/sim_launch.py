import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    GroupAction,
    ExecuteProcess,
    SetLaunchConfiguration,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    config_dir = get_package_share_directory("lunabot_config")

    nav2_params_file = os.path.join(
        config_dir, "params", "nav2", "nav2_test_bot_params.yaml"
    )

    declare_robot_mode = DeclareLaunchArgument(
        "robot_mode",
        default_value="manual",
        choices=["manual", "auto"],
        description="Select 'manual' for teleoperated mode or 'auto' for auto mode.",
    )

    declare_teleop_mode = DeclareLaunchArgument(
        "teleop_mode",
        default_value="keyboard",
        choices=["keyboard", "xbox"],
        description="Choose the teleoperation mode: 'keyboard' for keyboard control or 'xbox' for Xbox controller.",
    )

    declare_use_localization = DeclareLaunchArgument(
        "use_localization",
        default_value="false",
        choices=["true", "false"],
        description="Enable AprilTag localization phase before navigation (default: false).",
    )

    point_lio_config = os.path.join(
        config_dir,
        "params",
        "point_lio",
        "mid360.yaml",
    )

    ukf_params_file = os.path.join(
        config_dir, "params", "robot_localization", "ukf_params.yaml"
    )


    rtabmap_params_file = os.path.join(
        config_dir, "params", "rtabmap", "rtabmap_params.yaml"
    )

    bt_nav_to_pose = os.path.join(
        config_dir,
        "behavior_trees",
        "nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml",
    )

    bt_nav_through_poses = os.path.join(
        config_dir, "behavior_trees", "nav_through_poses_w_replanning_and_recovery.xml"
    )

    apriltag_params_file = os.path.join(
        config_dir, "params", "apriltag", "tag_params.yaml"
    )

    topic_remapper_node = Node(package="lunabot_util", executable="topic_remapper")

    # RGBD Synchronization nodes for dual-camera setup
    rgbd_sync_front = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync_front",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "approx_sync": True,
            "sync_queue_size": 1000,
        }],
        remappings=[
            ("rgb/image", "/camera_front/color/image_raw"),
            ("depth/image", "/camera_front/depth/image_rect_raw"),
            ("rgb/camera_info", "/camera_front/color/camera_info"),
            ("rgbd_image", "/camera_front/rgbd_image"),
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    rgbd_sync_back = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync_back",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "approx_sync": True,
            "sync_queue_size": 1000,
        }],
        remappings=[
            ("rgb/image", "/camera_back/color/image_raw"),
            ("depth/image", "/camera_back/depth/image_rect_raw"),
            ("rgb/camera_info", "/camera_back/color/camera_info"),
            ("rgbd_image", "/camera_back/rgbd_image"),
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
                "use_sim_time": True,
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
                "subscribe_scan_cloud": False,
                "subscribe_scan": False,
                "wait_imu_to_init": False,
                "subscribe_odom": True,
            },
            rtabmap_params_file,
        ],
        remappings=[
            ("rgbd_image0", "/camera_front/rgbd_image"),
            ("rgbd_image1", "/camera_back/rgbd_image"),
            ("odom", "/odometry/filtered"),
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
                "use_sim_time": True,
                "use_imu_as_input": False,
                "prop_at_freq_of_imu": True,
                "check_satu": False,
                "init_map_size": 10,
                "point_filter_num": 1,
                "space_down_sample": True,
                "filter_size_surf": 0.1,
                "filter_size_map": 0.1,
                "ivox_nearby_type": 26,
                "runtime_pos_log_enable": False,
                "publish.tf_send_en": False,
                "publish.odom_frame_id": "odom",
                "publish.base_frame_id": "base_link",
            },
        ],
        remappings=[
            ("/aft_mapped_to_init", "/lio_odom"),
        ],
    )

    ukf_node = Node(
        package="robot_localization",
        executable="ukf_node",
        name="ukf_filter_node",
        output="screen",
        parameters=[
            ukf_params_file,
            {"use_sim_time": True},
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

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        condition=LaunchConfigurationEquals("teleop_mode", "xbox"),
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[
            {
                "require_enable_button": False,
                "axis_linear.x": 1,
                "axis_angular.yaw": 0,
                "enable_turbo_button": 5,
                "scale_linear_turbo.x": 1.5,
                "scale_angular_turbo.yaw": 1.5,
            }
        ],
        condition=LaunchConfigurationEquals("teleop_mode", "xbox"),
    )

    keyboard_teleop_node = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--",
            "bash",
            "-c",
            "ros2 run lunabot_teleop keyboard_teleop.py; exit",
        ],
        output="screen",
        condition=LaunchConfigurationEquals("teleop_mode", "keyboard"),
    )

    apriltag_front_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        output="screen",
        parameters=[apriltag_params_file],
        remappings=[
            ("/image_rect", "/camera_front/color/image_raw"),
            ("/camera_info", "/camera_front/color/camera_info"),
        ],
    )

    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_mode)
    ld.add_action(declare_teleop_mode)
    ld.add_action(declare_use_localization)
    ld.add_action(topic_remapper_node)
    ld.add_action(rgbd_sync_front)
    ld.add_action(rgbd_sync_back)
    ld.add_action(apriltag_front_node)
    ld.add_action(map_to_odom_tf)

    # Manual mode: Event-based launch sequence
    # Step 1: Launch odometry stack when rgbd_sync starts
    manual_odometry_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=rgbd_sync_front,
            on_start=[point_lio_node, ukf_node],
        ),
        condition=LaunchConfigurationEquals("robot_mode", "manual"),
    )

    # Step 2: Launch SLAM when odometry starts
    manual_slam_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=point_lio_node,
            on_start=[slam_node],
        ),
        condition=LaunchConfigurationEquals("robot_mode", "manual"),
    )

    # Step 3: Launch Nav2 stack when SLAM starts
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

    # Step 4: Launch teleop when everything else is ready
    manual_teleop = GroupAction(
        actions=[joy_node, teleop_twist_joy_node, keyboard_teleop_node],
        condition=LaunchConfigurationEquals("robot_mode", "manual"),
    )

    ld.add_action(manual_odometry_handler)
    ld.add_action(manual_slam_handler)
    ld.add_action(manual_nav2_handler)
    ld.add_action(manual_teleop)

    # Auto mode: Event-based launch sequence
    # Step 1: Launch Nav2 stack immediately (needs to be ready first)
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

    # Step 2: When lifecycle manager starts, launch odometry/localization stack
    auto_localization_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_manager_node,
            on_start=[point_lio_node, ukf_node, slam_node],
        ),
        condition=LaunchConfigurationEquals("robot_mode", "auto"),
    )

    # Step 3: When SLAM starts, launch navigation clients
    auto_clients_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=slam_node,
            on_start=[excavation_server_node, navigation_client_node],
        ),
        condition=LaunchConfigurationEquals("robot_mode", "auto"),
    )

    ld.add_action(auto_nav2_stack)
    ld.add_action(auto_localization_handler)
    ld.add_action(auto_clients_handler)

    return ld
