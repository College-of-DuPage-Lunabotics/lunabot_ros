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
            "robot_description",
            Command(
                ["xacro ", urdf_file, " use_sim:=", LaunchConfiguration("use_sim")]
            ),
        )
    ]


def set_spawn_coordinates(context, *args, **kwargs):
    world_type = context.launch_configurations.get("arena_type")

    world_coords = {
        "ucf": {"x": "1.0", "y": "-3.0", "z": "0.35"},
        "artemis": {
            "x": "2.5",
            "y": "1.6",
            "z": "0.35",
        },
    }

    coords = world_coords.get(
        world_type, world_coords["artemis"]
    )

    return [
        SetLaunchConfiguration("spawn_x", coords["x"]),
        SetLaunchConfiguration("spawn_y", coords["y"]),
        SetLaunchConfiguration("spawn_z", coords["z"]),
    ]


def set_world_file(context, *args, **kwargs):
    sim_dir = get_package_share_directory("lunabot_sim")
    world_type = context.launch_configurations.get("arena_type")

    world_files = {
        "ucf": os.path.join(
            sim_dir, "worlds", "high_resolution", "ucf", "ucf_arena.world"
        ),
        "artemis": os.path.join(
            sim_dir, "worlds", "high_resolution", "artemis", "artemis_arena3.world"
        ),
    }

    world_file = world_files.get(world_type, world_files["ucf"])

    return [SetLaunchConfiguration("world_file", world_file)]


def generate_launch_description():
    config_dir = get_package_share_directory("lunabot_config")

    rviz_config_file = os.path.join(config_dir, "rviz", "robot_view.rviz")

    declare_robot_type = DeclareLaunchArgument(
        "robot_type",
        default_value="test",
        choices=["test", "bulldozer", "trencher"],
        description="Defines the robot configuration to use: 'test', 'bulldozer', or 'trencher', each with unique characteristics and capabilities.",
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

    declare_viz_type = DeclareLaunchArgument(
        "viz_type",
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

    declare_arena_type = DeclareLaunchArgument(
        "arena_type",
        default_value="artemis",
        choices=["ucf", "artemis"],
        description="Choose the arena: 'ucf' for UCF arena or 'artemis' for Artemis arena.",
    )

    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=LaunchConfigurationEquals("viz_type", "rviz"),
    )

    foxglove_bridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml",
            )
        ),
        condition=LaunchConfigurationEquals("viz_type", "foxglove"),
    )

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "gui": LaunchConfiguration("sim_gui"),
            "world": LaunchConfiguration("world_file"),
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
            LaunchConfiguration("spawn_x"),
            "-y",
            LaunchConfiguration("spawn_y"),
            "-Y",
            LaunchConfiguration("robot_heading"),
            "-z",
            LaunchConfiguration("spawn_z"),
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": LaunchConfiguration("robot_description"),
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
        condition=LaunchConfigurationEquals("robot_type", "bulldozer"),
    )

    blade_joint_controller_node = Node(
        package="lunabot_util",
        executable="blade_joint_controller",
        condition=LaunchConfigurationEquals("robot_type", "bulldozer"),
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
    ld.add_action(declare_viz_type)
    ld.add_action(declare_sim_gui)
    ld.add_action(declare_arena_type)

    ld.add_action(OpaqueFunction(function=set_orientation))
    ld.add_action(OpaqueFunction(function=set_robot_description))
    ld.add_action(OpaqueFunction(function=set_world_file))
    ld.add_action(OpaqueFunction(function=set_spawn_coordinates))

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
