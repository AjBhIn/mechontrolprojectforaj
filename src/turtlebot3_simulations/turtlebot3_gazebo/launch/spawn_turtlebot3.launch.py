# ==============================================================================
# Spawns /our_bot into Gazebo and initializes ROS-GZ Parameter Bridge
# Ensures spawn height z=0.05 to eliminate floor clipping physics explosions.
# ==============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    pkg_gazebo = get_package_share_directory('turtlebot3_gazebo')

    sdf_path = os.path.join(pkg_gazebo, 'models', model_folder, 'model.sdf')
    bridge_params = os.path.join(pkg_gazebo, 'params', model_folder + '_bridge.yaml')

    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')
    z_pose = LaunchConfiguration('z_pose', default='0.05')

    declare_x_cmd = DeclareLaunchArgument('x_pose', default_value='-2.0')
    declare_y_cmd = DeclareLaunchArgument('y_pose', default_value='-0.5')
    declare_z_cmd = DeclareLaunchArgument('z_pose', default_value='0.05')

    # Includes the dynamic Robot State Publisher for /our_bot
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'namespace': 'our_bot',
            'frame_prefix': 'our_bot/',
            'use_sim_time': 'true'
        }.items()
    )

    # Spawns /our_bot at z=0.05 height (avoids ground friction explosions)
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'our_bot',
            '-file', sdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen',
    )

    # ROS-Gazebo Parameter Bridge for /our_bot
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='our_bot_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # Image bridge for camera feed scoped to /our_bot namespace
    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/our_bot/camera/image_raw'],
        output='screen',
    )

    return LaunchDescription([
        declare_x_cmd,
        declare_y_cmd,
        declare_z_cmd,
        robot_state_publisher_cmd,
        start_gazebo_ros_spawner_cmd,
        start_gazebo_ros_bridge_cmd,
        start_gazebo_ros_image_bridge_cmd
    ])