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

    d456_front_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_front",
            "camera_namespace": "",
            "device_type": "d456",
            "serial_no": "'327522301245'",
            "publish_tf": "true",
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "2",
            "depth_module.profile": "640,480,15",
            "rgb_camera.profile": "640,480,15",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "depth_module.enable_auto_exposure": "true",
        }.items(),
    )

    d456_back_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_back",
            "camera_namespace": "",
            "device_type": "d456",
            "serial_no": "'333422300377'",
            "publish_tf": "true",
            "enable_gyro": "false",
            "enable_accel": "false",
            "depth_module.profile": "640,480,15",
            "rgb_camera.profile": "640,480,15",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "depth_module.enable_auto_exposure": "true",
        }.items(),
    )

    controller_teleop_node = Node(
        package="lunabot_teleop",
        executable="controller_teleop",
        name="controller_teleop",
        arguments=["--ros-args", "--log-level", "info"],
    )

    power_monitor_node = Node(
        package="lunabot_util",
        executable="power_monitor.py",
        name="power_monitor",
        output="screen",
        parameters=[
            {"serial_port": "/dev/ttyACM0"},
            {"baud_rate": 9600},
            {"publish_rate": 10.0},
        ],
    )
 
    apriltag_d456_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        output="screen",
        parameters=[
            apriltag_params_file,
            {"image_transport": "compressed"}
        ],
        remappings=[
            ("/image_rect", "/camera_front/color/image_raw"),
            ("/camera_info", "/camera_front/color/camera_info"),
        ],
    )

    livox_driver = Node(
        package="livox_ros_driver2",
        executable="livox_ros_driver2_node",
        name="livox_lidar_publisher",
        output="screen",
        parameters=[
            {"xfer_format": 1},  # 0 = PointCloud2 (PointXYZRTL), 1 = CustomMsg
            {"multi_topic": 0},  # 0 = single /livox/lidar topic
            {"data_src": 0},     # 0 = lidar data source
            {"publish_freq": 10.0},
            {"output_data_type": 0},
            {"frame_id": "livox_frame"},
            {"user_config_path": livox_params_file},
        ],
    )

    image_compressor_node = Node(
        package="lunabot_util",
        executable="image_compressor.py",
        name="image_compressor",
        output="screen",
        parameters=[
            {"jpeg_quality": 25},
            {"scale": 1.0},
        ],
    )

    rgbd_sync_front = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync_front",
        output="screen",
        parameters=[
            {"use_sim_time": False, "approx_sync": True, "sync_queue_size": 1000,
             "compressed_rate": 1.0,
             "rgb_image_transport": "compressed",
             "depth_image_transport": "compressedDepth"}
        ],
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
        parameters=[
            {"use_sim_time": False, "approx_sync": True, "sync_queue_size": 1000,
             "compressed_rate": 1.0,
             "rgb_image_transport": "compressed",
             "depth_image_transport": "compressedDepth"}
        ],
        remappings=[
            ("rgb/image", "/camera_back/color/image_raw"),
            ("depth/image", "/camera_back/depth/image_rect_raw"),
            ("rgb/camera_info", "/camera_back/color/camera_info"),
            ("rgbd_image", "/camera_back/rgbd_image"),
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    return LaunchDescription([
        livox_driver,
        image_compressor_node,
        d456_front_launch,
        d456_back_launch,
        controller_teleop_node,
        power_monitor_node,
        apriltag_d456_node,
        rgbd_sync_front,
        rgbd_sync_back,
    ])
