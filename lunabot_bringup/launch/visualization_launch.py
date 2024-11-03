import os
import xacro
from ament_index_python.packages import (
    get_package_share_path,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    simulation_dir = get_package_share_directory("lunabot_simulation")
    config_dir = get_package_share_directory("lunabot_config")

    urdf_file = os.path.join(simulation_dir, "urdf", "robot", "yahboom.xacro")
    rviz_config_file = os.path.join(config_dir, "rviz", "robot_view.rviz")

    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", urdf_file]), value_type=str
                )
            }
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": False}],
    )

    ld = LaunchDescription()

    ld.add_action(
        GroupAction(
            actions=[
                rviz_launch,
                robot_state_publisher_node,
                joint_state_publisher_node
            ]
        )
    )

    return ld