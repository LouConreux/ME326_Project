# Copyright 2022 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def launch_setup(context, *args, **kwargs):
    namespace_launch_arg = LaunchConfiguration('namespace')
    use_sim_time_launch_arg = LaunchConfiguration('use_sim_time')
    autostart_launch_arg = LaunchConfiguration('autostart')
    nav2_params_file_launch_arg = LaunchConfiguration('nav2_params_file')
    cmd_vel_topic_launch_arg = LaunchConfiguration('cmd_vel_topic')

    # Set env var to print messages to stdout immediately
    set_logging_env_var = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'recoveries_server',
        'bt_navigator',
        'waypoint_follower'
    ]

    remappings = [
        ('/cmd_vel', cmd_vel_topic_launch_arg),
    ]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time_launch_arg,
        'autostart': autostart_launch_arg,
        'local_costmap.local_costmap.ros__parameters.laser_layer.enabled':
            LaunchConfiguration('use_lidar'),
        'global_costmap.global_costmap.ros__parameters.laser_layer.enabled':
            LaunchConfiguration('use_lidar'),
    }

    configured_params = RewrittenYaml(
        source_file=nav2_params_file_launch_arg,
        root_key=namespace_launch_arg,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output={'both': 'screen'},
        parameters=[
            configured_params
        ],
        remappings=remappings,
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output={'both': 'screen'},
        parameters=[
            configured_params
        ],
        remappings=remappings,
    )

    recoveries_server_node = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output={'both': 'screen'},
        parameters=[
            configured_params
        ],
        remappings=remappings,
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output={'both': 'screen'},
        parameters=[
            configured_params
        ],
        remappings=remappings,
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output={'both': 'screen'},
        parameters=[
            configured_params
        ],
        remappings=remappings,
    )

    lifecycle_manager_navigation_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output={'both': 'screen'},
        parameters=[
            {'use_sim_time': use_sim_time_launch_arg},
            {'autostart': autostart_launch_arg},
            {'node_names': lifecycle_nodes},
        ],
    )

    return [
        set_logging_env_var,
        controller_server_node,
        planner_server_node,
        recoveries_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        lifecycle_manager_navigation_node,
    ]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_name',
            default_value='locobot',
            description='name of the robot (could be anything but defaults to `locobot`).',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_lidar',
            default_value='false',
            choices=('true', 'false'),
            description='if `true`, configure Nav2 to use lidar as an observation source.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='top-level namespace',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            choices=('true', 'false'),
            description='automatically startup the Nav2 stack.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value=(LaunchConfiguration('robot_name'), '/mobile_base/cmd_vel'),
            description='topic to remap /cmd_vel to.'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('interbotix_xslocobot_nav'),
                'config',
                'nav2_params.yaml'
            ]),
            description=(
                'full path to the ROS 2 parameters file to use when configuring the Nav2 stack.'
            ),
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=('true', 'false'),
            description=(
                'tells ROS nodes asking for time to get the Gazebo-published simulation time, '
                'published over the ROS topic /clock; this value is automatically set to `true` if'
                ' using Gazebo hardware.'
            )
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
