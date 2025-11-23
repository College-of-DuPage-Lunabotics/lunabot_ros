import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    SetLaunchConfiguration,
    TimerAction,
    GroupAction,
    ExecuteProcess,
    OpaqueFunction,
)


def set_nav2_params(context, *args, **kwargs):
    robot_type = context.launch_configurations.get("robot_type")
    config_dir = get_package_share_directory("lunabot_config")

    nav2_params_file = os.path.join(
        config_dir, "params", "nav2", f"nav2_{robot_type}_bot_params.yaml"
    )
    return [SetLaunchConfiguration("nav2_params_file", nav2_params_file)]


def generate_launch_description():
    config_dir = get_package_share_directory("lunabot_config")

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

    declare_robot_type = DeclareLaunchArgument(
        "robot_type",
        default_value="bulldozer",
        choices=["bulldozer", "trencher"],
        description="Specify the type of robot to launch: 'bulldozer', or 'trencher'. Each option loads the respective robot configuration.",
    )

    ukf_params_file = os.path.join(
        config_dir, "params", "robot_localization", "ukf_params.yaml"
    )

    kiss_icp_config = os.path.join(
        get_package_share_directory("lunabot_config"),
        "params",
        "kiss_icp",
        "mid360.yaml",
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

    slam_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="log",
        parameters=[
            {
                "use_sim_time": True,
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

    kiss_icp_node = Node(
        package="kiss_icp",
        executable="kiss_icp_node",
        name="kiss_icp_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True
            },
            kiss_icp_config,
        ],
        remappings=[
            ("pointcloud_topic", "/livox/lidar"),
        ],
    )

    ukf_node = Node(
        package="robot_localization",
        executable="ukf_node",
        name="ukf_filter_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
            },
            ukf_params_file,
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
    )

    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[LaunchConfiguration("nav2_params_file")],
    )

    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[LaunchConfiguration("nav2_params_file")],
    )

    behavior_server_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[LaunchConfiguration("nav2_params_file")],
    )

    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            LaunchConfiguration("nav2_params_file"),
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

    apriltag_d456_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        output="screen",
        parameters=[apriltag_params_file],
        remappings=[
            ("/image_rect", "/d456/color/image_raw"),
            ("/camera_info", "/d456/color/camera_info"),
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
    ld.add_action(declare_robot_type)
    ld.add_action(OpaqueFunction(function=set_nav2_params))
    ld.add_action(topic_remapper_node)
    ld.add_action(apriltag_d456_node)
    ld.add_action(map_to_odom_tf)

    ld.add_action(
        GroupAction(
            actions=[
                TimerAction(
                    period=2.0,
                    actions=[
                        ukf_node,
                        kiss_icp_node,
                    ],
                ),
                TimerAction(
                    period=5.0,
                    actions=[
                        slam_node,
                    ],
                ),
                TimerAction(
                    period=20.0,
                    actions=[
                        controller_server_node,
                        planner_server_node,
                        behavior_server_node,
                        bt_navigator_node,
                        lifecycle_manager_node,
                    ],
                ),
                joy_node,
                teleop_twist_joy_node,
                keyboard_teleop_node,
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
                        # localization_server_node,
                        navigation_client_node,
                    ],
                ),
                TimerAction(
                    period=5.0,
                    actions=[
                        ukf_node,
                        slam_node,
                        kiss_icp_node,
                    ],
                ),
                TimerAction(
                    period=10.0,
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
