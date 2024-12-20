import os
import random
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    OpaqueFunction,
    GroupAction,
)


def set_orientation(context, *args, **kwargs):
    orientations = {"north": -1.5708, "east": 3.1416, "south": 1.5708, "west": 0.0}
    random_orientation = random.choice(list(orientations.values()))
    chosen_orientation = context.launch_configurations.get("robot_heading")

    if chosen_orientation == "random":
        return [SetLaunchConfiguration("robot_heading", str(random_orientation))]
    else:
        return [
            SetLaunchConfiguration(
                "robot_heading", str(orientations[chosen_orientation])
            )
        ]


def set_robot_description(context, *args, **kwargs):
    robot_type = context.launch_configurations.get("robot_type")
    config_dir = get_package_share_directory("lunabot_description")

    urdf_file = os.path.join(config_dir, "urdf", f"{robot_type}_bot.urdf.xacro")
    return [
        SetLaunchConfiguration(
            "robot_description", Command(["xacro ", urdf_file, " use_sim:=", LaunchConfiguration("use_sim")])
        )
    ]


def generate_launch_description():
    sim_dir = get_package_share_directory("lunabot_sim")
    config_dir = get_package_share_directory("lunabot_config")

    rviz_config_file = os.path.join(config_dir, "rviz", "robot_view.rviz")

    world_file = os.path.join(
        sim_dir, "worlds", "high_resolution", "artemis", "artemis_arena2.world"
    )

    declare_robot_type = DeclareLaunchArgument(
        "robot_type",
        default_value="bulldozer",
        choices=["bulldozer", "trencher"],
        description="Defines the robot configuration to use: 'bulldozer', or 'trencher', each with unique characteristics and capabilities.",
    )

    declare_use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="true",
        description="Specifies whether the robot is in simulation mode 'true' or real-world mode 'false'.",
    )

    declare_robot_heading = DeclareLaunchArgument(
        "robot_heading",
        default_value="east",
        choices=["north", "east", "south", "west", "random"],
        description="Sets the starting orientation of the robot. Choose a cardinal direction ('north', 'east', 'south', 'west') or 'random' for a randomized orientation.",
    )

    declare_vis_type = DeclareLaunchArgument(
        "vis_type",
        default_value="rviz",
        choices=["rviz", "foxglove"],
        description="Choose 'rviz' for visualization in RViz or 'foxglove' for visualization in Foxglove Studio.",
    )

    declare_sim_gui = DeclareLaunchArgument(
        "sim_gui",
        default_value="true",
        choices=["true", "false"],
        description="Sets whether to open Gazebo with its GUI. 'true' opens the GUI, while 'false' runs Gazebo in headless mode.",
    )

    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=LaunchConfigurationEquals("vis_type", "rviz"),
    )

    foxglove_bridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml",
            )
        ),
        condition=LaunchConfigurationEquals("vis_type", "foxglove"),
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "gui": LaunchConfiguration("sim_gui"),
            "world": world_file,
        }.items(),
    )

    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            LaunchConfiguration("robot_type"),
            "-x",
            "2.5",
            "-y",
            "1.6",
            "-Y",
            LaunchConfiguration("robot_heading"),
            "-z",
            "0.35",
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": LaunchConfiguration(
                    "robot_description"
                ),
                "use_sim_time": LaunchConfiguration("use_sim"),
            }
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

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "position_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=LaunchConfigurationNotEquals("robot_type", "trencher"),
    )

    blade_joint_controller_node = Node(
        package="lunabot_util",
        executable="blade_joint_controller",
        condition=LaunchConfigurationNotEquals("robot_type", "trencher"),
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

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": False}],
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_type)
    ld.add_action(declare_use_sim)
    ld.add_action(declare_robot_heading)
    ld.add_action(declare_vis_type)
    ld.add_action(declare_sim_gui)

    ld.add_action(OpaqueFunction(function=set_orientation))
    ld.add_action(OpaqueFunction(function=set_robot_description))

    ld.add_action(rviz_launch)
    ld.add_action(foxglove_bridge_launch)
    ld.add_action(robot_state_publisher)

    ld.add_action(
        GroupAction(
            actions=[
                sim_launch,
                spawn_robot_node,
                joint_state_broadcaster_spawner,
                diff_drive_controller_spawner,
                position_controller_spawner,
                blade_joint_controller_node,
            ],
            condition=LaunchConfigurationEquals("use_sim", "true"),
        )
    )

    ld.add_action(
        GroupAction(
            actions=[
                joint_state_publisher_node,
            ],
            condition=LaunchConfigurationEquals("use_sim", "false"),
        )
    )

    return ld
