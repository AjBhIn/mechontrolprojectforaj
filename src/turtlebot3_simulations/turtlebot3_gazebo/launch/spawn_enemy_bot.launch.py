# ==============================================================================
# Spawns /enemy_bot into Gazebo and initializes ROS-GZ Parameter Bridge
# ==============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    sdf_path = os.path.join(pkg_gazebo, 'models', 'turtlebot3_waffle', 'model_enemy_bot.sdf')
    bridge_params = os.path.join(pkg_gazebo, 'params', 'enemy_bot_bridge.yaml')

    x_pose = LaunchConfiguration('x_pose', default='2.0')
    y_pose = LaunchConfiguration('y_pose', default='0.5')
    z_pose = LaunchConfiguration('z_pose', default='0.05')
    yaw_pose = LaunchConfiguration('yaw_pose', default='3.14159')

    declare_x_cmd = DeclareLaunchArgument('x_pose', default_value='2.0')
    declare_y_cmd = DeclareLaunchArgument('y_pose', default_value='0.5')
    declare_z_cmd = DeclareLaunchArgument('z_pose', default_value='0.05')
    declare_yaw_cmd = DeclareLaunchArgument('yaw_pose', default_value='3.14159')

    # Single dynamic inclusion of Robot State Publisher for /enemy_bot
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'namespace': 'enemy_bot',
            'frame_prefix': 'enemy_bot/',
            'use_sim_time': 'true'
        }.items()
    )

    spawner_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'enemy_bot',
            '-file', sdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose,
            '-yaw', yaw_pose
        ],
        output='screen',
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='enemy_bot_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_x_cmd,
        declare_y_cmd,
        declare_z_cmd,
        declare_yaw_cmd,
        robot_state_publisher_cmd,
        spawner_node,
        bridge_node
    ])