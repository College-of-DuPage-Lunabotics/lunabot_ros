import os
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
    GroupAction,
)


def generate_launch_description():
    simulation_dir = get_package_share_directory("lunabot_simulation")
    config_dir = get_package_share_directory("lunabot_config")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    nav2_params_file = os.path.join(config_dir, "params", "nav2_params.yaml")
    rtabmap_params_file = os.path.join(config_dir, "params", "rtabmap_params.yaml")

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
                "subscribe_rgb": False,
                "subscribe_odom_info": False,
                "odom_sensor_sync": True,
                "frame_id": "base_link",
                "map_frame_id": "map",
                "odom_frame_id": "odom",
                "publish_tf": True,
                "publish_tf_odom": False,
                "database_path": "",
                "approx_sync": True,
                "sync_queue_size": 1000,
                "subscribe_scan_cloud": False,
                "subscribe_scan": True,
                "wait_imu_to_init": True,
                "rgb_topic": "/color/image_raw",
                "depth_topic": "/depth/image_raw",
                "camera_info_topic": "/color/camera_info",
                "scan_topic": "/scan",
            },
            rtabmap_params_file,
        ],
        arguments=["--ros-args", "--log-level", "error"],
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

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
    )

    driver_node = Node(
        package="lunabot_driver",
        executable="mecanum_driver",
    )

    astra_node = Node(
        package="astra_camera",
        executable="astra_camera_node",
        name="camera",
        output="screen",
        parameters=[
            {
                "camera_name": "camera",
                "depth_registration": False,
                "serial_number": "",
                "device_num": 1,
                "vendor_id": 0,
                "product_id": 0,
                "enable_point_cloud": True,
                "enable_colored_point_cloud": False,
                "point_cloud_qos": "default",
                "connection_delay": 100,
                "color_width": 640,
                "color_height": 480,
                "color_fps": 30,
                "enable_color": True,
                "flip_color": False,
                "color_qos": "default",
                "color_camera_info_qos": "default",
                "depth_width": 640,
                "depth_height": 480,
                "depth_fps": 30,
                "enable_depth": True,
                "flip_depth": False,
                "depth_qos": "default",
                "depth_camera_info_qos": "default",
                "ir_width": 640,
                "ir_height": 480,
                "ir_fps": 30,
                "enable_ir": True,
                "flip_ir": False,
                "ir_qos": "default",
                "ir_camera_info_qos": "default",
                "publish_tf": False,
                "tf_publish_rate": 10.0,
                "ir_info_url": "",
                "color_info_url": "",
                "color_roi_x": -1,
                "color_roi_y": -1,
                "color_roi_width": -1,
                "color_roi_height": -1,
                "depth_roi_x": -1,
                "depth_roi_y": -1,
                "depth_roi_width": -1,
                "depth_roi_height": -1,
                "depth_scale": 1,
                "color_depth_synchronization": False,
                "use_uvc_camera": True,
                "uvc_vendor_id": 0x2bc5,
                "uvc_product_id": 0x0501,
                "uvc_retry_count": 100,
                "uvc_camera_format": "mjpeg",
                "uvc_flip": False,
                "oni_log_level": "verbose",
                "oni_log_to_console": False,
                "oni_log_to_file": False,
                "enable_d2c_viewer": False,
                "enable_publish_extrinsic": False,
            }
        ],
    )

    s3_lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": "/dev/ttyUSB0",
                "serial_baudrate": 1000000,
                "frame_id": "laser_link",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "DenseBoost",
            }
        ],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(
        GroupAction(
            actions=[
                slam_node,
                nav2_launch,
                joy_node,
                driver_node,
                astra_node,
                s3_lidar_node,
            ]
        )
    )

    return ld
