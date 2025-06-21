# FILE 2: Save this as 'scenario_1_gazebo_nav.launch.py' in your 'launch/' directory.
# This is the master launch file that starts Gazebo and Nav2.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches a full Gazebo simulation with TIAGo, Nav2, and choreographed humans
    for a social navigation validation scenario.
    """
    my_pkg_share = get_package_share_directory('sm_mppi_planner')
    tiago_nav_share = get_package_share_directory('tiago_navigation')
    tiago_gazebo_share = get_package_share_directory('tiago_gazebo')
    
    # --- Path to the map file for Nav2 ---
    map_filepath = os.path.join(my_pkg_share, 'maps', 'hallway_map.yaml')

    # --- Add our custom models to Gazebo's search path ---
    gazebo_model_path = os.path.join(my_pkg_share, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ':' + gazebo_model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    # ======================== SCENARIO 1 CONFIGURATION ========================
    startup_delay_seconds = 7.0
    
    humans_yaml_string = """
- {type: standing, x: -8.0, y: 1.0, vx: 0.7, vy: 0.0}
- {type: sitting,  x: 8.0, y: -1.0, vx: -0.45, vy: 0.0}
"""
    # =========================================================================

    # --- ACTION 1: Launch Gazebo with TIAGo ---
    tiago_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tiago_gazebo_share, 'launch', 'tiago_gazebo.launch.py')),
        launch_arguments={'use_sim_time': 'true', 'world_name': 'empty'}.items()
    )

    # --- ACTION 2: Launch TIAGo's Nav2 Stack ---
    tiago_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tiago_nav_share, 'launch', 'tiago_navigation.launch.py')),
        launch_arguments={'map': map_filepath, 'use_sim_time': 'true'}.items()
    )

    # --- ACTION 3: Spawn Your Custom World and Human Models ---
    spawn_hallway_node = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'hallway', '-file', os.path.join(my_pkg_share, 'models', 'hallway_world', 'model.sdf')],
        output='screen'
    )
    
    # Spawn a standing human model
    spawn_human_0 = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'human_0', '-file', os.path.join(my_pkg_share, 'models', 'social_human', 'model.sdf'), '-x', '-8.0', '-y', '1.0'],
        output='screen'
    )
    
    # Spawn a wheelchair model
    spawn_human_1 = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'human_1', '-file', os.path.join(my_pkg_share, 'models', 'social_wheelchair', 'model.sdf'), '-x', '8.0', '-y', '-1.0'],
        output='screen'
    )
    
    # --- ACTION 4: Launch Your Gazebo Human Controller ---
    gazebo_human_controller_node = Node(
        package='sm_mppi_planner',
        executable='gazebo_human_controller', # Use the new script name
        name='gazebo_human_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'humans_yaml': humans_yaml_string,
            'startup_delay': startup_delay_seconds,
            'x_limits': [-10.0, 10.0],
            'y_limits': [-2.5, 2.5]
        }]
    )

    # --- ACTION 5: Launch RViz ---
    rviz_config_file = os.path.join(my_pkg_share, 'rviz', 'nav2_config.rviz') # Use a Nav2-specific config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        # Declare arguments if you want them to be configurable from the command line
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Launch all components
        tiago_gazebo_launch,
        tiago_navigation_launch,
        spawn_hallway_node,
        spawn_human_0,
        spawn_human_1,
        gazebo_human_controller_node,
        rviz_node
    ])
