#!/usr/bin/env python3
"""
Actions Launch
Starts excavation and dumping action servers
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Whether we are in simulation or not",
    )

    # In simulation, use sim-compatible servers (control bucket via position controller)
    sim_servers = GroupAction(
        actions=[
            Node(
                package="lunabot_nav",
                executable="excavation_server_sim",
                name="excavation_server",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="lunabot_nav",
                executable="dumping_server_sim",
                name="dumping_server",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="lunabot_nav",
                executable="homing_server_sim",
                name="homing_server",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ],
        condition=LaunchConfigurationEquals("use_sim", "true"),
    )

    # For real hardware, use hardware servers (control bucket via CAN motors)
    real_servers = GroupAction(
        actions=[
            Node(
                package="lunabot_nav",
                executable="excavation_server",
                name="excavation_server",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package="lunabot_nav",
                executable="dumping_server",
                name="dumping_server",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            Node(
                package="lunabot_nav",
                executable="homing_server",
                name="homing_server",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
        ],
        condition=LaunchConfigurationEquals("use_sim", "false"),
    )

    return LaunchDescription([
        declare_use_sim,
        sim_servers,
        real_servers,
    ])
