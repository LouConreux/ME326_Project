from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='',
            description='Name of the robot (default: empty)'
        ),
        Node(
            package='interbotix_xslocobot_control',  
            executable='bartender.py',
            name='bartender_node',
            output='screen',
            parameters=[{'robot_name': LaunchConfiguration('robot_name')}]
        )
    ])