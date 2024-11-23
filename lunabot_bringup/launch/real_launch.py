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
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    realsense_dir = get_package_share_directory("realsense2_camera")

    nav2_params_file = os.path.join(
        config_dir, "params", "nav2", "nav2_real_bot_params.yaml"
    )
    rtabmap_params_file = os.path.join(
        config_dir, "params", "rtabmap", "rtabmap_params.yaml"
    )
    ukf_params_file = os.path.join(
        config_dir, "params", "robot_localization", "ukf_params.yaml"
    )
    s3_params_file = os.path.join(
        config_dir, "params", "laser_filters", "s3_params.yaml"
    )

    declare_robot_mode = DeclareLaunchArgument(
        "robot_mode", default_value="manual", choices=["manual", "autonomous"]
    )

    rgbd_sync1_node = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync1",
        output="screen",
        parameters=[
            {"use_sim_time": False, "approx_sync": True, "sync_queue_size": 1000}
        ],
        remappings=[
            ("rgb/image", "/d456/color/image_raw"),
            ("depth/image", "/d456/depth/image_rect_raw"),
            ("rgb/camera_info", "/d456/color/camera_info"),
            ("rgbd_image", "/d456/rgbd_image"),
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
            {"use_sim_time": False, "approx_sync": True, "sync_queue_size": 1000}
        ],
        remappings=[
            ("rgb/image", "/d455/color/image_raw"),
            ("depth/image", "/d455/depth/image_rect_raw"),
            ("rgb/camera_info", "/d455/color/camera_info"),
            ("rgbd_image", "/d455/rgbd_image"),
        ],
        namespace="d455",
        arguments=["--ros-args", "--log-level", "error"],
    )

    slam_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
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
                "imu_topic": "/d456/imu/data",
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
                "freq": 20.0,
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
                "use_sim_time": False,
            },
            ukf_params_file,
        ],
    )

    localization_server_node = Node(
        package="lunabot_system",
        executable="localization_server",
        name="localization_server",
        output="screen",
    )

    navigation_client_node = Node(
        package="lunabot_system",
        executable="navigation_client",
        name="navigation_client",
        output="screen",
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": nav2_params_file,
        }.items(),
    )

    s3_lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": "/dev/ttyUSB0",
                "serial_baudrate": 1000000,
                "frame_id": "s3_lidar_link",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "DenseBoost",
                "scan_frequency": 20.0,
            }
        ],
        output="screen",
    )

    s3_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[s3_params_file],
        remappings=[("scan", "/scan_raw"), ("scan_filtered", "/scan")],
    )

    d455_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "camera_name": "d455",
            "camera_namespace": "",
            "device_type": "d455",
            "publish_tf": "true",
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "2",
            "depth_module.depth_profile": "640x480x60",
            "rgb_camera.color_profile": "640x480x60",
        }.items(),
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
            "unite_imu_method2": "2",
            "depth_module.depth_profile": "640x480x60",
            "rgb_camera.color_profile": "640x480x60",
        }.items(),
    )

    d455_imu_filter = Node(
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
            ("imu/data_raw", "/d455/imu/data_raw"),
            ("imu/data", "/d455/imu/data"),
        ],
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

    imu_rotator_node = Node(package="lunabot_system", executable="imu_rotator")

    robot_controller_node = Node(
        package="lunabot_system",
        executable="robot_controller",
        parameters=[
            {
                "xbox_mode": True,
                "outdoor_mode": False,
            }
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
    ld.add_action(rgbd_sync1_node)
    ld.add_action(rgbd_sync2_node)
    ld.add_action(s3_lidar_node)
    ld.add_action(s3_filter_node)
    ld.add_action(d455_launch)
    ld.add_action(d456_launch)
    ld.add_action(imu_rotator_node)
    ld.add_action(d455_imu_filter)
    ld.add_action(d456_imu_filter)
    ld.add_action(robot_controller_node)
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
                        nav2_launch,
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
                    period=5.0,
                    actions=[
                        localization_server_node,
                        navigation_client_node,
                    ],
                ),
                TimerAction(
                    period=50.0,
                    actions=[
                        icp_odometry_node,
                        rf2o_odometry_node,
                        ukf_node,
                        slam_node,
                    ],
                ),
                TimerAction(
                    period=60.0,
                    actions=[
                        nav2_launch,
                    ],
                ),
            ],
            condition=LaunchConfigurationEquals("robot_mode", "autonomous"),
        )
    )

    return ld
