# File: src/sm_mppi_planner/launch/validation.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_sm_mppi_planner = get_package_share_directory('sm_mppi_planner')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Path to the new validation world
    world_file = os.path.join(pkg_sm_mppi_planner, 'worlds', 'validation.world')

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items(),
    )

    return LaunchDescription([
        gazebo
    ])