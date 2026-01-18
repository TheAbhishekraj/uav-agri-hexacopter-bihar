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

    # Dynamic Path Resolution (Best Practice)
    # Note: Ensure 'hexacopter_control' is built and sourced, or GAZEBO_MODEL_PATH is set.
    # If running from source without install, we rely on the environment variables set in launch_split.sh
    # but here we define the standard ROS 2 way to find assets.
    pkg_share = FindPackageShare('hexacopter_control')
    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'bihar_farm.world'])
    model_path = PathJoinSubstitution([pkg_share, 'models', 'hexacopter_agricultural', 'model.sdf'])

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
