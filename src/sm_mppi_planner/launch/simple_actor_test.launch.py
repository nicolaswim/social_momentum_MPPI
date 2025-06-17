# sm_mppi_planner/launch/simple_actor_test.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_sm_mppi_planner = get_package_share_directory('sm_mppi_planner')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value=os.path.join(pkg_sm_mppi_planner, 'worlds', 'wim_test.world'),
        description='Full path to the world file to load')

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world_name')}.items(),
    )

    rviz_node = ExecuteProcess(
        cmd=['bash', '-c', f'LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 rviz2 -d {os.path.join(pkg_sm_mppi_planner, "rviz", "actor_test.rviz")} --ros-args --params-file /tmp/launch_params_s2bzpyn8'],
        output='screen'
    )
    
    # Human Aggregator Node
    human_aggregator_node = Node(
        package='sm_mppi_planner',
        executable='human_aggregator_node', # The name of the script
        name='human_aggregator_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'trajectory_file': os.path.join(pkg_sm_mppi_planner, 'config', 'human_trajectories.yaml')}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_name_arg,
        gazebo,
        rviz_node,
        human_aggregator_node,
    ])