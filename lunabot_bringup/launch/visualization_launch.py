import os
import random
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    OpaqueFunction,
    GroupAction,
)


def set_orientation(context, *args, **kwargs):
    orientations = {"north": -1.5708, "east": 3.1416, "south": -1.5708, "west": 0.0}
    random_orientation = random.choice(list(orientations.values()))
    chosen_orientation = context.launch_configurations.get("orientation")

    if chosen_orientation == "random":
        return [SetLaunchConfiguration("robot_orientation", str(random_orientation))]
    else:
        return [
            SetLaunchConfiguration(
                "robot_orientation", str(orientations[chosen_orientation])
            )
        ]


def generate_launch_description():
    simulation_dir = get_package_share_directory("lunabot_simulation")
    config_dir = get_package_share_directory("lunabot_config")

    rviz_config_file = os.path.join(config_dir, "rviz", "robot_view.rviz")
    
    urdf_simulation_file = os.path.join(
        simulation_dir, "urdf", "robot", "rectangle_bot.xacro"
    )

    world_file = os.path.join(
        simulation_dir,
        "urdf",
        "worlds",
        "artemis",
        "artemis_arena.world",
    )

    rviz_config_file = os.path.join(config_dir, "rviz", "robot_view.rviz")

    robot_simulation_description = Command(["xacro ", urdf_simulation_file])

    declare_orientation = DeclareLaunchArgument(
        "orientation",
        default_value="east",
        choices=["north", "east", "south", "west", "random"],
    )

    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={"world": world_file}.items(),
    )

    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "rectangle_bot",
            "-x",
            "2.5",
            "-y",
            "1.6",
            "-Y",
            LaunchConfiguration("robot_orientation"),
            "-z",
            "0.35",
        ],
        output="screen",
    )

    robot_sim_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_simulation_description, "use_sim_time": True}
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    blade_joint_controller_node = Node(
        package="lunabot_simulation", executable="blade_joint_controller"
    )

    ld = LaunchDescription()

    ld.add_action(
        GroupAction(
            actions=[
                declare_orientation,
                OpaqueFunction(function=set_orientation),
                rviz_launch,
                gazebo_launch,
                spawn_robot_node,
                robot_sim_state_publisher,
                joint_state_broadcaster_spawner,
                diff_drive_controller_spawner,
                position_controller_spawner,
                blade_joint_controller_node,
            ],
        )
    )

    return ld
