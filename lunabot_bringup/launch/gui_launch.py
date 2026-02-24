import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    OpaqueFunction,
    GroupAction,
)


def set_orientation(context, *args, **kwargs):
    orientations = {"north": -1.5708, "east": 3.1416, "south": 1.5708, "west": 0.0}
    chosen_orientation = context.launch_configurations.get("robot_heading")
    return [SetLaunchConfiguration("robot_heading", str(orientations.get(chosen_orientation, 0.0)))]


def set_spawn_coordinates(context, *args, **kwargs):
    world_coords = {
        "ucf": {"x": "1.0", "y": "-3.0", "z": "0.35"},
        "artemis": {"x": "2.5", "y": "1.6", "z": "0.35"},
    }
    world_type = context.launch_configurations.get("arena_type")
    coords = world_coords.get(world_type, world_coords["artemis"])
    return [
        SetLaunchConfiguration("spawn_x", coords["x"]),
        SetLaunchConfiguration("spawn_y", coords["y"]),
        SetLaunchConfiguration("spawn_z", coords["z"]),
    ]


def set_world_file(context, *args, **kwargs):
    sim_dir = get_package_share_directory("lunabot_sim")
    world_type = context.launch_configurations.get("arena_type")
    world_files = {
        "ucf": os.path.join(sim_dir, "worlds", "high_resolution", "ucf", "ucf_arena.world"),
        "artemis": os.path.join(sim_dir, "worlds", "high_resolution", "artemis", "artemis_arena.world"),
    }
    return [SetLaunchConfiguration("world_file", world_files.get(world_type, world_files["artemis"]))]


def generate_launch_description():
    config_dir = get_package_share_directory("lunabot_config")
    description_dir = get_package_share_directory("lunabot_description")

    rviz_config_file = os.path.join(config_dir, "rviz", "robot_view.rviz")
    urdf_file = os.path.join(description_dir, "urdf", "v1_bot.urdf.xacro")
    gui_params_file = os.path.join(
        config_dir, "params", "gui_params.yaml"
    )

    declare_use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="true",
        description="Specifies whether the robot is in simulation mode 'true' or real-world mode 'false'.",
    )

    declare_robot_heading = DeclareLaunchArgument(
        "robot_heading",
        default_value="north",
        choices=["north", "east", "south", "west"],
        description="Sets the starting orientation of the robot. Choose a cardinal direction.",
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

    declare_viz_mode = DeclareLaunchArgument(
        "viz_mode",
        default_value="gui",
        choices=["rviz", "gui"],
        description="Choose visualization mode: 'rviz' for RViz2 or 'gui' for custom PyQt GUI.",
    )

    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    custom_gui_node = Node(
        package="lunabot_gui",
        executable="lunabot_gui",
        name="lunabot_gui",
        parameters=[
            gui_params_file,
            {'mode': LaunchConfiguration('use_sim', default='true')},  # Pass use_sim as mode (true=sim, false=real)
        ],
        output="screen",
    )

    topic_remapper_node = Node(
        package="lunabot_util",
        executable="topic_remapper"
    )

    bandwidth_monitor_node = Node(
        package="lunabot_util",
        executable="bandwidth_monitor.py",
        name="bandwidth_monitor",
        output="screen",
        parameters=[{'interface': 'wlo1'}],  # Force WiFi interface
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    ["xacro ", urdf_file, " use_sim:=", LaunchConfiguration("use_sim")]
                ),
                "use_sim_time": LaunchConfiguration("use_sim"),
            }
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": False}],
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
            "v2_bot",
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

    # Only launch action servers in sim mode - in real mode they're started with hardware
    actions_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("lunabot_bringup"), "launch", "actions_launch.py"
            )
        ),
        launch_arguments={
            "use_sim": LaunchConfiguration("use_sim"),
        }.items(),
        condition=LaunchConfigurationEquals("use_sim", "true"),
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

    camera_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "camera_controller",
            "--controller-manager",
            "/controller_manager",
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

    sim_group = GroupAction(
        actions=[
            sim_launch,
            spawn_robot_node,
            topic_remapper_node,
            image_compressor_node,
            joint_state_broadcaster_spawner,
            diff_drive_controller_spawner,
            position_controller_spawner,
            camera_controller_spawner,
        ],
        condition=LaunchConfigurationEquals("use_sim", "true"),
    )

    joint_state_publisher_real = GroupAction(
        actions=[joint_state_publisher_node],
        condition=LaunchConfigurationEquals("use_sim", "false"),
    )

    # Joy node for controller input in real mode
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
    )

    joy_group = GroupAction(
        actions=[joy_node],
        condition=LaunchConfigurationEquals("use_sim", "false"),
    )

    rviz_group = GroupAction(
        actions=[rviz_launch],
        condition=LaunchConfigurationEquals("viz_mode", "rviz"),
    )

    custom_gui_group = GroupAction(
        actions=[custom_gui_node],
        condition=LaunchConfigurationEquals("viz_mode", "gui"),
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim)
    ld.add_action(declare_robot_heading)
    ld.add_action(declare_sim_gui)
    ld.add_action(declare_arena_type)
    ld.add_action(declare_viz_mode)

    ld.add_action(OpaqueFunction(function=set_orientation))
    ld.add_action(OpaqueFunction(function=set_world_file))
    ld.add_action(OpaqueFunction(function=set_spawn_coordinates))

    ld.add_action(rviz_group)
    ld.add_action(custom_gui_group)
    ld.add_action(robot_state_publisher)
    ld.add_action(bandwidth_monitor_node)
    ld.add_action(sim_group)
    ld.add_action(joint_state_publisher_real)
    ld.add_action(joy_group)
    ld.add_action(actions_launch)

    return ld