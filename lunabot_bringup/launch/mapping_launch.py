import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_dir = get_package_share_directory("lunabot_config")
    
    rtabmap_params_file = os.path.join(
        config_dir, "params", "rtabmap", "rtabmap_params.yaml"
    )

    declare_use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Whether we are in simulation or not",
    )

    rgbd_sync_front = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync_front",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim"),
                "approx_sync": True,
                "sync_queue_size": 1000,
            }
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
            {
                "use_sim_time": LaunchConfiguration("use_sim"),
                "approx_sync": True,
                "sync_queue_size": 1000,
            }
        ],
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
                "use_sim_time": LaunchConfiguration("use_sim"),
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
            ("rgbd_image0", "/camera_front/rgbd_image"),
            ("rgbd_image1", "/camera_back/rgbd_image"),
            ("odom", "/odometry/filtered"),
            ("scan_cloud", "/livox/lidar_PointCloud2"),
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    return LaunchDescription([
        declare_use_sim,
        rgbd_sync_front,
        rgbd_sync_back,
        slam_node,
    ])
