import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_dir = get_package_share_directory("lunabot_config")
    
    bt_nav_to_pose = os.path.join(
        config_dir,
        "behavior_trees",
        "nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml",
    )
    bt_nav_through_poses = os.path.join(
        config_dir, "behavior_trees", "nav_through_poses_w_replanning_and_recovery.xml"
    )

    declare_use_sim = DeclareLaunchArgument(
        "use_sim",
        default_value="false",
        description="Whether we are in simulation or not",
    )
    
    def launch_setup(context, *args, **kwargs):
        use_sim = LaunchConfiguration("use_sim").perform(context)
        
        if use_sim == "true":
            nav2_params_file = os.path.join(
                config_dir, "params", "nav2", "nav2_v1_bot_params.yaml"
            )
        else:
            nav2_params_file = os.path.join(
                config_dir, "params", "nav2", "nav2_real_bot_params.yaml"
            )
        
        use_sim_time = use_sim == "true"

        controller_server_node = Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
        )

        planner_server_node = Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
        )

        behavior_server_node = Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
        )

        bt_navigator_node = Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                nav2_params_file,
                {
                    "use_sim_time": use_sim_time,
                    "default_nav_to_pose_bt_xml": bt_nav_to_pose,
                    "default_nav_through_poses_bt_xml": bt_nav_through_poses,
                },
            ],
        )

        lifecycle_manager_node = Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": True},
                {
                    "node_names": [
                        "controller_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                    ]
                },
                {"node_timeout": 10.0},
            ],
        )

        return [
            controller_server_node,
            planner_server_node,
            behavior_server_node,
            bt_navigator_node,
            lifecycle_manager_node,
        ]

    return LaunchDescription([
        declare_use_sim,
        OpaqueFunction(function=launch_setup)
    ])
