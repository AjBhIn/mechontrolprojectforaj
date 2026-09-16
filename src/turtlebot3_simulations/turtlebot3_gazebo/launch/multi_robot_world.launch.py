# ==============================================================================
# Unified Multi-Robot World Launch File
# Runs a single Gazebo server/client instance and spawns both robots at z=0.05
# ==============================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('turtlebot3_gazebo')
    launch_dir = os.path.join(pkg_gazebo, 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = os.path.join(pkg_gazebo, 'worlds', 'turtlebot3_world.world')

    # Single Gazebo Server & Client instance
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    # Spawn Robot 1 (/our_bot) at x=-2.0, y=-0.5, z=0.05
    spawn_our_bot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': '-2.0', 'y_pose': '-0.5', 'z_pose': '0.05'}.items()
    )

    # Spawn Robot 2 (/enemy_bot) at x=2.0, y=0.5, z=0.05, facing 180 degrees (3.14159 rad)
    spawn_enemy_bot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'spawn_enemy_bot.launch.py')),
        launch_arguments={
            'x_pose': '2.0',
            'y_pose': '0.5',
            'z_pose': '0.05',
            'yaw_pose': '3.14159'
        }.items()
    )

    set_env_vars = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_gazebo, 'models')
    )

    # Add this node to your return LaunchDescription([]) array
    global_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='global_clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        spawn_our_bot,
        spawn_enemy_bot,
        set_env_vars,
        global_clock
    ])