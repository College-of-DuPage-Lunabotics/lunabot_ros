#!/usr/bin/env python3
"""
Launch file for Lunabot GUI package
Starts the PyQt5 GUI and bandwidth monitor
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for GUI"""
    
    # Declare launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='Robot mode: sim or real (affects system control availability)'
    )
    
    interface_arg = DeclareLaunchArgument(
        'interface',
        default_value='auto',
        description='Network interface to monitor (auto/eth0/enp0s31f6/etc)'
    )
    
    warning_threshold_arg = DeclareLaunchArgument(
        'warning_threshold',
        default_value='8.0',
        description='Warning threshold in Mbps'
    )
    
    error_threshold_arg = DeclareLaunchArgument(
        'error_threshold',
        default_value='10.0',
        description='Error threshold in Mbps'
    )
    
    # Bandwidth monitor node
    bandwidth_monitor_node = Node(
        package='lunabot_util',
        executable='bandwidth_monitor.py',
        name='bandwidth_monitor',
        parameters=[{
            'interface': LaunchConfiguration('interface'),
            'warning_threshold': LaunchConfiguration('warning_threshold'),
            'error_threshold': LaunchConfiguration('error_threshold'),
            'update_rate': 1.0,
        }],
        output='screen'
    )
    
    # GUI node
    gui_node = Node(
        package='lunabot_gui',
        executable='lunabot_gui',
        name='lunabot_gui',
        parameters=[{
            'mode': LaunchConfiguration('mode'),
        }],
        output='screen',
    )
    
    return LaunchDescription([
        mode_arg,
        interface_arg,
        warning_threshold_arg,
        error_threshold_arg,
        bandwidth_monitor_node,
        gui_node,
    ])
