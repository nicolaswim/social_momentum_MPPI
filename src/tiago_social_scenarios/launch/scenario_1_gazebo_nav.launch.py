import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches a full Gazebo simulation with TIAGo, the standard Nav2 stack,
    and choreographed humans for social navigation validation.
    """
    
    # --- Package Directories ---
    my_pkg_share = get_package_share_directory('sm_mppi_planner')
    
    # Correctly find the necessary TIAGo packages
    tiago_gazebo_pkg_share = get_package_share_directory('tiago_gazebo')
    tiago_nav_pkg_share = get_package_share_directory('tiago_2dnav') #<-- CORRECTED PACKAGE

    # --- RViz Configuration for Nav2 ---
    # It's best to use the default Nav2 rviz config provided by PAL
    rviz_config_file = os.path.join(tiago_nav_pkg_share, 'config', 'rviz', 'navigation.rviz')

    # --- Map file for Nav2 ---
    map_filepath = os.path.join(my_pkg_share, 'maps', 'hallway_map.yaml')

    # --- Add our custom models to Gazebo's search path ---
    gazebo_model_path = os.path.join(my_pkg_share, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ':' + gazebo_model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    # ===================================================================================
    # ========================== SCENARIO CONFIGURATION =================================
    # ===================================================================================
    
    startup_delay_seconds = 7.0
    
    humans_yaml_string = """
- {type: standing, x: -8.0, y: 1.0, vx: 0.7, vy: 0.0}
- {type: sitting,  x: 8.0, y: -1.0, vx: -0.45, vy: 0.0}
"""
    # ===================================================================================

    # --- ACTION 1: Launch Gazebo with TIAGo ---
    tiago_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tiago_gazebo_share, 'launch', 'tiago_gazebo.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'world_name': 'empty',
            'is_public_sim': 'True' # This is required
        }.items()
    )

    # --- ACTION 2: Launch TIAGo's Nav2 Stack ---
    tiago_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # THIS IS THE CORRECTED PATH
            os.path.join(tiago_nav_pkg_share, 'launch', 'tiago_nav_bringup.launch.py')
        ),
        launch_arguments={
            'map': map_filepath,
            'use_sim_time': 'true',
            'params_file': os.path.join(tiago_nav_pkg_share, 'config', 'tiago_pmb2_nav_public_sim.yaml')
        }.items()
    )

    # --- ACTION 3: Spawn Your Custom World and Human Models ---
    spawn_hallway_node = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'hallway_world', '-file', os.path.join(my_pkg_share, 'models', 'hallway_world', 'model.sdf'), '-x', '0', '-y', '0'],
        output='screen'
    )
    
    spawn_human_0 = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'human_0', '-file', os.path.join(my_pkg_share, 'models', 'social_human', 'model.sdf'), '-x', '-8.0', '-y', '1.0'],
        output='screen'
    )
    
    spawn_human_1 = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-entity', 'human_1', '-file', os.path.join(my_pkg_share, 'models', 'social_wheelchair', 'model.sdf'), '-x', '8.0', '-y', '-1.0'],
        output='screen'
    )
    
    # --- ACTION 4: Launch Your Gazebo Human Controller ---
    gazebo_human_controller_node = Node(
        package='sm_mppi_planner',
        executable='gazebo_human_controller',
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
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        tiago_gazebo_launch,
        tiago_navigation_launch,
        spawn_hallway_node,
        spawn_human_0,
        spawn_human_1,
        gazebo_human_controller_node,
        rviz_node
    ])