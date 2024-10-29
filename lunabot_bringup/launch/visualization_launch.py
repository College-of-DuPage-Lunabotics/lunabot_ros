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
    orientations = {"north": -1.5708, "east": 3.1416, "south": -1.5708, "west": 0.0}
    random_orientation = random.choice(list(orientations.values()))
    chosen_orientation = context.launch_configurations.get("robot_orientation")

    if chosen_orientation == "random":
        return [SetLaunchConfiguration("robot_orientation", str(random_orientation))]
    else:
        return [
            SetLaunchConfiguration(
                "robot_orientation", str(orientations[chosen_orientation])
            )
        ]

def set_robot_description(context, *args, **kwargs):
    robot_type = context.launch_configurations.get("robot_type")
    simulation_dir = get_package_share_directory("lunabot_simulation")
    
    urdf_files = {
        "rectangle": os.path.join(simulation_dir, "urdf", "robot", "simulation", "rectangle_bot.xacro"),
        "square": os.path.join(simulation_dir, "urdf", "robot", "simulation", "square_bot.xacro"),
        "trencher": os.path.join(simulation_dir, "urdf", "robot", "simulation", "trencher_bot.xacro")
    }
    
    urdf_file = urdf_files.get(robot_type)
    return [SetLaunchConfiguration("robot_simulation_description", Command(["xacro ", urdf_file]))]

def generate_launch_description():
    simulation_dir = get_package_share_directory("lunabot_simulation")
    config_dir = get_package_share_directory("lunabot_config")

    rviz_config_file = os.path.join(config_dir, "rviz", "robot_view.rviz")
    urdf_real_file = os.path.join(simulation_dir, "urdf", "robot", "real", "trencher_bot.xacro")
    world_file = os.path.join(simulation_dir, "urdf", "worlds", "high_resolution", "artemis", "artemis_arena.world")

    declare_robot_type = DeclareLaunchArgument(
        "robot_type",
        default_value="rectangle",
        choices=["rectangle", "square", "trencher"],
        description="Defines the robot configuration to use: 'rectangle', 'square', or 'trencher', each with unique characteristics and capabilities."
    )

    declare_visualization_mode = DeclareLaunchArgument(
        "visualization_mode",
        default_value="simulation",
        choices=["simulation", "real"],
        description="Selects the visualization context: 'simulation' for a simulated environment or 'real' for real-world robot visualization."
    )

    declare_robot_orientation = DeclareLaunchArgument(
        "robot_orientation",
        default_value="east",
        choices=["north", "east", "south", "west", "random"],
        description="Sets the starting orientation of the robot. Choose a cardinal direction ('north', 'east', 'south', 'west') or 'random' for a randomized orientation."
    )

    declare_visualization_type = DeclareLaunchArgument(
        "visualization_type",
        default_value="rviz",
        choices=["rviz", "foxglove"],
        description="Choose 'rviz' for visualization in RViz or 'foxglove' for visualization in Foxglove Studio."
    )

    declare_gazebo_gui = DeclareLaunchArgument(
        "gazebo_gui",
        default_value="true",
        choices=["true", "false"],
        description="Sets whether to open Gazebo with its GUI. 'true' opens the GUI, while 'false' runs Gazebo in headless mode."
    )

    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=LaunchConfigurationEquals("visualization_type", "rviz"),
    )

    foxglove_bridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"
            )
        ),
        condition=LaunchConfigurationEquals("visualization_type", "foxglove"),
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gazebo_gui"),
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
            {"robot_description": LaunchConfiguration("robot_simulation_description"), "use_sim_time": True}
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
        package="lunabot_simulation",
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

    robot_real_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": Command(["xacro ", urdf_real_file]), "use_sim_time": False}
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": False}],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_type)
    ld.add_action(declare_visualization_mode)
    ld.add_action(declare_robot_orientation)
    ld.add_action(declare_visualization_type)
    ld.add_action(declare_gazebo_gui)

    ld.add_action(OpaqueFunction(function=set_orientation))
    ld.add_action(OpaqueFunction(function=set_robot_description))

    ld.add_action(rviz_launch)
    ld.add_action(foxglove_bridge_launch)

    ld.add_action(
        GroupAction(
            actions=[
                gazebo_launch,
                spawn_robot_node,
                robot_sim_state_publisher,
                joint_state_broadcaster_spawner,
                diff_drive_controller_spawner,
                position_controller_spawner,
                blade_joint_controller_node,
            ],
            condition=LaunchConfigurationEquals("visualization_mode", "simulation"),
        )
    )

    # Real mode group
    ld.add_action(
        GroupAction(
            actions=[
                joy_node,
                robot_real_state_publisher_node,
                joint_state_publisher_node,
            ],
            condition=LaunchConfigurationEquals("visualization_mode", "real"),
        )
    )

    return ld
