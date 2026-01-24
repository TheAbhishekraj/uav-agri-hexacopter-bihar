# =================================================================
# Launch File: launch_drone.launch.py
# Purpose: Orchestrates Gazebo Sim and spawns the Hexacopter model
# =================================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Find the ROS Gazebo Sim package
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Dynamic Path Resolution: Uses FindPackageShare to locate assets relative to the install prefix
    # This replaces hardcoded /home/user paths for portability.
    pkg_share = FindPackageShare('hexacopter_control')
    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'bihar_farm.sdf'])
    model_path = PathJoinSubstitution([pkg_share, 'models', 'hexacopter_agricultural', 'model.sdf'])

    # 1. Include the standard Gazebo Sim launch description
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 2. Spawn our Blue Bird (The Hexacopter)
    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'hexacopter',
            '-file', model_path,
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen',
    )

    return LaunchDescription([
        gz_sim,
        spawn_drone,
    ])
