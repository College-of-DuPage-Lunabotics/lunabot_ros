import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
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
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

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

    rtabmap_params_file = os.path.join(
        config_dir, "params", "rtabmap", "rtabmap_params.yaml"
    )

    s3_params_file = os.path.join(
        config_dir, "params", "laser_filters", "s3_params.yaml"
    )
    s2l_params_file = os.path.join(
        config_dir, "params", "laser_filters", "s2l_params.yaml"
    )

    ukf_params_file = os.path.join(
        config_dir, "params", "robot_localization", "ukf_params.yaml"
    )

    apriltag_params_file = os.path.join(
        config_dir, "params", "apriltag", "tag_params.yaml"
    )

    topic_remapper_node = Node(package="lunabot_util", executable="topic_remapper")

    rgbd_sync1_node = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync1",
        output="screen",
        parameters=[
            {"use_sim_time": True, "approx_sync": True, "sync_queue_size": 1000}
        ],
        remappings=[
            ("rgb/image", "/d456/color/image_raw"),
            ("depth/image", "/d456/depth/image_rect_raw"),
            ("rgb/camera_info", "/d456/color/camera_info"),
            ("rgbd_image", "rgbd_image"),
        ],
        namespace="d456",
        arguments=["--ros-args", "--log-level", "error"],
    )

    rgbd_sync2_node = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync2",
        output="screen",
        parameters=[
            {"use_sim_time": True, "approx_sync": True, "sync_queue_size": 1000}
        ],
        remappings=[
            ("rgb/image", "/d455/color/image_raw"),
            ("depth/image", "/d455/depth/image_rect_raw"),
            ("rgb/camera_info", "/d455/color/camera_info"),
            ("rgbd_image", "rgbd_image"),
        ],
        namespace="d455",
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
                "rgbd_cameras": 2,
                "subscribe_depth": False,
                "subscribe_rgbd": True,
                "subscribe_rgb": False,
                "subscribe_odom_info": False,
                "odom_sensor_sync": True,
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "odom",
                "publish_tf": False,
                "publish_tf_odom": False,
                "database_path": "",
                "approx_sync": True,
                "sync_queue_size": 1000,
                "subscribe_scan_cloud": False,
                "subscribe_scan": True,
                "wait_imu_to_init": True,
                "imu_topic": "/teensy/imu/data",
            },
            rtabmap_params_file,
        ],
        remappings=[
            ("rgbd_image0", "/d456/rgbd_image"),
            ("rgbd_image1", "/d455/rgbd_image"),
            ("scan", "/scan"),
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    icp_odometry_node = Node(
        package="rtabmap_odom",
        executable="icp_odometry",
        output="screen",
        parameters=[
            {
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "publish_tf": False,
                "approx_sync": True,
                "Reg/Strategy": "1",
                "Odom/Strategy": "1",
                "Odom/FilteringStrategy": "1",
                "Odom/KalmanProcessNoise": "0.001",
                "Odom/KalmanMeasurementNoise": "0.01",
                "Icp/PointToPlane": "true",
                "Icp/Iterations": "10",
                "Icp/VoxelSize": "0.1",
                "Icp/Epsilon": "0.001",
                "Icp/PointToPlaneK": "20",
                "Icp/PointToPlaneRadius": "0",
                "Icp/MaxTranslation": "2",
                "Icp/MaxCorrespondenceDistance": "1",
                "Icp/Strategy": "1",
                "Icp/OutlierRatio": "0.7",
                "Icp/CorrespondenceRatio": "0.01",
                "Odom/ScanKeyFrameThr": "0.4",
                "OdomF2M/ScanSubtractRadius": "0.1",
                "OdomF2M/ScanMaxSize": "15000",
                "OdomF2M/BundleAdjustment": "false",
            }
        ],
        remappings=[
            ("scan", "/scan"),
            ("odom", "/icp_odom"),
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    rf2o_odometry_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[
            {
                "laser_scan_topic": "/scan",
                "odom_topic": "/rf2o_odom",
                "publish_tf": False,
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 40.0,
            }
        ],
        arguments=["--ros-args", "--log-level", "error"],
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

    s3_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[s3_params_file],
        remappings=[("scan", "/scan_raw"), ("scan_filtered", "/scan")],
    )

    s2l_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[s2l_params_file],
        remappings=[("scan", "/scan2_raw"), ("scan_filtered", "/scan2")],
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

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": LaunchConfiguration("nav2_params_file"),
        }.items(),
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

    apriltag_node = Node(
            package='apriltag_ros', executable='apriltag_node', output='screen',
            parameters=[apriltag_params_file],
            remappings=[('/image_rect', '/d455/color/image_raw'),
                        ('/camera_info', '/d455/color/camera_info')])

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
    ld.add_action(rgbd_sync1_node)
    ld.add_action(rgbd_sync2_node)
    ld.add_action(apriltag_node)
    ld.add_action(map_to_odom_tf)

    ld.add_action(
        GroupAction(
            actions=[
                TimerAction(
                    period=2.0,
                    actions=[
                        icp_odometry_node,
                        rf2o_odometry_node,
                        ukf_node,
                        s3_filter_node,
                        s2l_filter_node,
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
                        nav2_launch,
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
                        #localization_server_node,
                        navigation_client_node,
                    ],
                ),
                TimerAction(
                    period=5.0,
                    actions=[
                        icp_odometry_node,
                        rf2o_odometry_node,
                        ukf_node,
                        slam_node,
                        s3_filter_node,
                        s2l_filter_node,
                    ],
                ),
                TimerAction(
                    period=10.0,
                    actions=[
                        nav2_launch,
                    ],
                ),
            ],
            condition=LaunchConfigurationEquals("robot_mode", "auto"),
        )
    )

    return ld
