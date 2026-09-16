#!/usr/bin/env python3
# ==============================================================================
# Standalone & Dynamic Robot State Publisher Launch File
# Dynamically accepts 'namespace' and 'frame_prefix' arguments.
# Remaps transform outputs to global /tf and /tf_static channels for ROS 2.
# ==============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
    urdf_file_name = f'turtlebot3_{TURTLEBOT3_MODEL}.urdf'

    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Dynamic arguments passed from calling spawn launch files
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace', default='our_bot')
    frame_prefix = LaunchConfiguration('frame_prefix', default='our_bot/')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('namespace', default_value='our_bot', description='Target robot namespace'),
        DeclareLaunchArgument('frame_prefix', default_value='our_bot/', description='Frame prefix for TF frames'),

        # RSP Node: Scoped under the specified namespace with explicit global TF remappings
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
                'frame_prefix': frame_prefix
            }],
            remappings=[
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ]
        ),
    ])