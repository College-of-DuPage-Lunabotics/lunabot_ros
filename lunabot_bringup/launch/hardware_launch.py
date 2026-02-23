import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    config_dir = get_package_share_directory("lunabot_config")
    realsense_dir = get_package_share_directory("realsense2_camera")

    apriltag_params_file = os.path.join(
        config_dir, "params", "apriltag", "tag_params.yaml"
    )
    livox_params_file = os.path.join(config_dir, "params", "mid360", "mid360.json")

    # RealSense D456 Front Camera
    d456_front_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "camera_name": "d456_front",
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

    # RealSense D456 Back Camera
    d456_back_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "camera_name": "d456_back",
            "camera_namespace": "",
            "device_type": "d456",
            "publish_tf": "true",
            "enable_gyro": "false",
            "enable_accel": "false",
            "depth_module.depth_profile": "640x480x60",
            "rgb_camera.color_profile": "640x480x60",
        }.items(),
    )

    # IMU Complementary Filter
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

    # Controller Teleop
    controller_teleop_node = Node(
        package="lunabot_teleop",
        executable="controller_teleop",
        name="controller_teleop",
        arguments=["--ros-args", "--log-level", "info"],
    )

    # AprilTag Detection
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

    # Livox Mid-360 LiDAR
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

    # RGBD Sync nodes for RTAB-Map
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

    return LaunchDescription([
        livox_driver,
        d456_front_launch,
        d456_back_launch,
        d456_imu_filter,
        controller_teleop_node,
        apriltag_d456_node,
        rgbd_sync_front,
        rgbd_sync_back,
    ])
