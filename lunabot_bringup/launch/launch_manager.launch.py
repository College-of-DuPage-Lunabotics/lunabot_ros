from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lunabot_bringup',
            executable='launch_manager_node.py',
            name='launch_manager',
            output='screen',
            parameters=[],
            emulate_tty=True
        )
    ])
