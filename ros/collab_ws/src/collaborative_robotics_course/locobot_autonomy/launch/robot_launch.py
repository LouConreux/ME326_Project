#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    recording_duration_arg = DeclareLaunchArgument(
        'recording_duration',
        default_value='5.0',
        description='Duration for audio recording in seconds'
    )
    
    # Define nodes
    speech_processor_node = Node(
        package='locobot_autonomy',
        executable='speech.py',
        name='speech_processor',
        output='screen',
        parameters=[{
            'recording_duration': LaunchConfiguration('recording_duration')
        }]
    )
    
    main_coordinator_node = Node(
        package='locobot_autonomy',
        executable='main.py',
        name='main_coordinator',
        output='screen'
    )
    
    perception_node = Node(
        package='locobot_autonomy',
        executable='perception.py',
        name='perception_node',
        output='screen'
    )
    
    # New task queue manager node
    task_queue_manager_node = Node(
        package='locobot_autonomy',
        executable='task_queue_manager.py',
        name='task_queue_manager',
        output='screen'
    )
    
    navigation_node = Node(
        package='locobot_autonomy',
        executable='navigate.py',
        name='navigation_node',
        output='screen'
    )
    
    manipulation_node = Node(
        package='locobot_autonomy',
        executable='manipulation.py',
        name='manipulation_node',
        output='screen'
    )
    
    return LaunchDescription([
        recording_duration_arg,
        speech_processor_node,
        main_coordinator_node,
        perception_node,
        task_queue_manager_node,  # Added this new node
        navigation_node,
        manipulation_node
    ])