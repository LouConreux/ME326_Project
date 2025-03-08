#!/usr/bin/env python3
"""
Unified launch file for the Locobot perception and speech command system.
This launches components in separate terminals for better log visibility.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for combined speech and perception system with separate terminals."""
    return LaunchDescription([
        # Speech processing node in its own terminal
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'locobot_autonomy', 'speech.py'],
            name='speech_processor_terminal',
            output='screen'
        ),
        
        # Perception node in its own terminal
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'ros2', 'run', 'locobot_autonomy', 'perception.py'],
            name='perception_node_terminal',
            output='screen'
        ),
        
        # Main coordinator node in the launch terminal
        Node(
            package='locobot_autonomy',
            executable='main.py',
            name='main_coordinator',
            output='screen',
            emulate_tty=True,
        )
    ])