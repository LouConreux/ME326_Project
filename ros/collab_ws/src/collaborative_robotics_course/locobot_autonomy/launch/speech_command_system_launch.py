#!/usr/bin/env python3
"""
Launch file for the speech command system.
This launches the speech processor and main coordinator nodes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for speech command system."""
    return LaunchDescription([
        Node(
            package='locobot_autonomy',  # Replace with your actual package name
            executable='speech_processor.py',
            name='speech_processor',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'json_key_path': '/home/locobot/Group1/ME326_Project/loulou_key.json',  # Update this path
                'recording_duration': 5.0
            }]
        ),
        Node(
            package='locobot_autonomy',  # Replace with your actual package name
            executable='main_coordinator.py',
            name='main_coordinator',
            output='screen',
            emulate_tty=True,
        ),
        # You can add additional nodes like perception and task execution here
    ])