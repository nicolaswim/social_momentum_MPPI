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
    This launch file starts the TIAGo simulation and the social MPPI planner.
    All key parameters are defined in a simple 'SETTINGS' block at the top.
    """
    
    tiago_gazebo_pkg_share_dir = ""
    try:
        tiago_gazebo_pkg_share_dir = get_package_share_directory('tiago_gazebo')
    except PackageNotFoundError:
        print("[ERROR] Package 'tiago_gazebo' not found. Gazebo will not be launched.")

    # ===================================================================================
    # ======================== EASY SETTINGS CONFIGURATION BLOCK ========================
    # ===================================================================================

    # --- Navigation Goal ---
    goal = {'x': 5.0, 'y': 0.0}

    # --- Human Simulation ---
    standing_human_mesh_path = 'package://sm_mppi_planner/models/Slampion/Slampion.dae'
    sitting_human_mesh_path = 'package://sm_mppi_planner/models/wheelchair/wheelchair.dae' 
    
    
    human_simulation_params = {
        'num_random_humans': 3,
        'people_standing_up': 1, 
        'x_limits': [-5.0, 5.0],
        'y_limits': [-5.0, 5.0],
        'human_max_speed': 0.3,
        'human_min_speed': 0.1,
    }

    # --- General Simulation Settings ---
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    world_name_arg = DeclareLaunchArgument('world_name', default_value='empty')
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='true')
    launch_gazebo_arg = DeclareLaunchArgument('launch_gazebo', default_value='true')
        
    # ===================================================================================
    # ============== END OF SETTINGS - NO NEED TO EDIT BELOW THIS LINE ==================
    # ===================================================================================

    # --- Gazebo Simulation ---
    tiago_simulation_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tiago_gazebo_pkg_share_dir, 'launch', 'tiago_gazebo.launch.py')
                ),
                launch_arguments={
                    'is_public_sim': 'True',
                    'world_name': LaunchConfiguration('world_name'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
                condition=IfCondition(PythonExpression([f"'{tiago_gazebo_pkg_share_dir}' != ''"]))
            )
        ]
    )

    # --- RViz ---
    rviz_with_ld_preload_action = ExecuteProcess(
        cmd=['bash', '-c', f"LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true"],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    # --- Your Nodes ---
    sm_mppi_planner_node = Node(
        package='sm_mppi_planner',
        executable='mppi_planner_node',
        name='sm_mppi_planner_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    goal_publisher_node = Node(
        package='sm_mppi_planner',
        executable='goal_publisher_node',
        name='goal_publisher_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'goal_x': goal['x']},
            {'goal_y': goal['y']}
        ]
    )

    # Combine the human settings with other necessary parameters
    human_publisher_full_params = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'world_frame_id': 'odom',
        'publish_frequency': 10.0,
        'humans_topic': '/humans',
        'markers_topic': '/human_markers',
        'standing_human_mesh_path': standing_human_mesh_path, 
        'sitting_human_mesh_path': sitting_human_mesh_path, 
    }
    human_publisher_full_params.update(human_simulation_params)

    human_publisher_node = Node(
        package='sm_mppi_planner',
        executable='fake_human_publisher',
        name='fake_human_publisher',
        output='screen',
        parameters=[human_publisher_full_params]
    )

    # --- Assemble the Launch Description ---
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(world_name_arg)
    ld.add_action(launch_rviz_arg)
    ld.add_action(launch_gazebo_arg)
    ld.add_action(tiago_simulation_group)
    ld.add_action(rviz_with_ld_preload_action)
    ld.add_action(sm_mppi_planner_node)
    ld.add_action(goal_publisher_node)
    ld.add_action(human_publisher_node)

    return ld