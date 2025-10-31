import os
from os import environ, pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_pal.include_utils import include_scoped_launch_py_description

def get_model_paths(packages_names):
    model_paths = ""
    for package_name in packages_names:
        if model_paths != "":
            model_paths += pathsep
        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, "share")
        model_paths += model_path
    if 'GAZEBO_MODEL_PATH' in environ:
        model_paths += pathsep + environ['GAZEBO_MODEL_PATH']
    return model_paths

def generate_launch_description():
    # --- args ---
    world_arg = DeclareLaunchArgument('world_name', default_value='my_world',
                                      description='Gazebo world name (no .world)')
    base_arg  = DeclareLaunchArgument('base_type',  default_value='pmb2',
                                      description='Robot base type')

    # --- packages/paths ---
    social_pkg       = get_package_share_directory('tiago_social_scenarios')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    tiago_gazebo_pkg = get_package_share_directory('tiago_gazebo')
    tiago_bringup_pkg= get_package_share_directory('tiago_bringup')

    map_yaml   = os.path.join(social_pkg, 'maps', 'hallway_map.yaml')
    nav2_params= os.path.join(nav2_bringup_pkg, 'params', 'nav2_params.yaml')  # <- stock Nav2 params
    rviz_cfg   = os.path.join(nav2_bringup_pkg, 'rviz', 'nav2_default_view.rviz')

    # =========================================================================
    # Gazebo startup: IDENTICAL to your original
    # =========================================================================
    packages = ['tiago_description', 'pmb2_description',
                'pal_hey5_description', 'pal_gripper_description',
                'pal_robotiq_description', 'omni_base_description']
    model_path = get_model_paths(packages)
    gazebo_model_path_env_var = SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path)

    start_gazebo_world_cmd = include_scoped_launch_py_description(
        pkg_name='pal_gazebo_worlds',
        paths=['launch', 'pal_gazebo.launch.py'],
        env_vars=[gazebo_model_path_env_var],
        launch_arguments={
            "world_name":  LaunchConfiguration('world_name'),
            "model_paths": packages,
            "resource_paths": packages,
        })
    # =========================================================================

    # Spawn robot
    spawn_tiago_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tiago_gazebo_pkg, 'launch', 'robot_spawn.launch.py')),
        launch_arguments={'robot_name': 'tiago',
                          'base_type': LaunchConfiguration('base_type')}.items()
    )

    # Bringup (sim time)
    start_tiago_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tiago_bringup_pkg, 'launch', 'tiago_bringup.launch.py')),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    # Controller names: adjust BASE_CTRL if needed (often 'diff_drive_base_controller')
    JOINT_BC = 'joint_state_broadcaster'
    BASE_CTRL = 'diff_drive_base_controller'   # try this first; TIAGo often uses diff_drive

    spawner_js = Node(
        package='controller_manager', executable='spawner',
        arguments=[
            JOINT_BC,
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
            '--wait-for',
        ],
        output='screen'
    )

    spawner_base = Node(
        package='controller_manager', executable='spawner',
        arguments=[
            BASE_CTRL,
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
            '--wait-for',
        ],
        output='screen'
    )

    # give Gazebo + spawn plenty of time before spawners run
    start_ctrls = TimerAction(period=15.0, actions=[spawner_js, spawner_base])


    # Nav2 (stock params to avoid pal_navigation_cfg_utils) + your map + RViz
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': 'True',
            'params_file': nav2_params
        }.items()
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav2_params
        }.items()
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'rviz_launch.py')),
        launch_arguments={'rviz_config': rviz_cfg}.items()
    )

    ld = LaunchDescription()
    ld.add_action(world_arg)
    ld.add_action(base_arg)
    ld.add_action(start_gazebo_world_cmd)   # unchanged Gazebo
    ld.add_action(spawn_tiago_cmd)
    ld.add_action(start_ctrls)              # ensure /odom
    ld.add_action(start_tiago_bringup_cmd)
    ld.add_action(localization)
    ld.add_action(navigation)
    ld.add_action(rviz)
    
    return ld