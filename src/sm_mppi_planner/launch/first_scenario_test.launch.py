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
    
    # --- Package Directories ---
    pkg_share_dir = get_package_share_directory('sm_mppi_planner')
    tiago_gazebo_pkg_share_dir = ""
    try:
        tiago_gazebo_pkg_share_dir = get_package_share_directory('tiago_gazebo')
    except PackageNotFoundError:
        print("[ERROR] Package 'tiago_gazebo' not found. Gazebo will not be launched.")

    # --- RViz Configuration ---
    rviz_config_file = os.path.join(pkg_share_dir, 'rviz', 'hallway_test.rviz')

    # ===================================================================================
    # ======================== EASY SETTINGS CONFIGURATION BLOCK ========================
    # ===================================================================================

    # --- Hallway Configuration (Centralized) ---
    hallway_params = {
        'hallway_length': 20.0,
        'hallway_width': 5.0,
        'wall_thickness': 0.2,
        'wall_height': 2.5,
        'wall_mesh_path': 'package://sm_mppi_planner/models/wall/wall.dae'
    }

    # --- Navigation Goal ---
    goal = {'x': 18.0, 'y': 0.0} # Example goal near the end of the hallway

    # --- Human Simulation ---
    standing_human_mesh_path = 'package://sm_mppi_planner/models/Slampion/Slampion.dae'
    sitting_human_mesh_path = 'package://sm_mppi_planner/models/wheelchair/wheelchair.dae' 
    
    human_simulation_params = {
        'total_people': 4,
        'people_standing_up': 4,
        # Dynamically set human limits based on hallway dimensions
        'x_limits': [-(hallway_params['hallway_length']/2 - 1.0), (hallway_params['hallway_length']/2 - 1.0)],
        'y_limits': [-(hallway_params['hallway_width']/2 - 1.0), (hallway_params['hallway_width']/2 - 1.0)],
        'human_max_speed': 0.3,
        'human_min_speed': 0.1,
    }

    # --- DYNAMICALLY GENERATE OBSTACLE STRING FOR PLANNER ---
    # This ensures the planner always knows the correct wall locations
    hw = hallway_params['hallway_width'] / 2.0
    hl = hallway_params['hallway_length'] / 2.0
    wt = hallway_params['wall_thickness'] / 2.0

    hallway_walls_yaml_string = f"""
    [
        [{-hl}, {hw + wt}, {hl}, {hw + wt}, {hl}, {hw - wt}, {-hl}, {hw - wt}],
        [{-hl}, {-hw - wt}, {hl}, {-hw - wt}, {hl}, {-hw + wt}, {-hl}, {-hw + wt}]
    ]
    """

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
        cmd=['bash', '-c', f"LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 rviz2 -d {rviz_config_file} --ros-args -p use_sim_time:=true"],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    # --- Your Nodes ---
    sm_mppi_planner_node = Node(
        package='sm_mppi_planner',
        executable='mppi_planner_node',
        name='sm_mppi_planner_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'static_obstacles_yaml': hallway_walls_yaml_string}
        ]
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

    # UPDATED: Pass the hallway parameters to the publisher node
    hallway_publisher_node = Node(
        package='sm_mppi_planner',
        executable='hallway_publisher',
        name='hallway_publisher',
        output='screen',
        parameters=[hallway_params]
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
    ld.add_action(hallway_publisher_node)

    return ld