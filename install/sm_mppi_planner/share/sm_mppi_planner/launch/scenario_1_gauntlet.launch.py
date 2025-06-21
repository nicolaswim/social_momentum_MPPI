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
    SCENARIO 1: The Bidirectional Flow Gauntlet (Revised)
    Tests navigation in a dense, two-way corridor with heterogeneous agents.
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
    # ======================== SCENARIO 1 SETTINGS BLOCK ================================
    # ===================================================================================

    # --- Hallway Configuration (Centralized) ---
    hallway_params = {
        'hallway_length': 20.0,
        'hallway_width': 5.0,
        'wall_thickness': 0.1,
        'wall_height': 2.5,
        'wall_mesh_path': 'package://sm_mppi_planner/models/wall/wall.dae'
    }

    startup_delay_seconds = 5.0

    # --- Navigation Goal ---
    goal = {'x': 9.0, 'y': 0.0}

    # --- Planner Configuration ---
    planner_params = {
        'static_cost_weight': 25.0,
        'safety_boundary': 0.2
    }
    
#     # --- Human Choreography (YAML String) ---
    humans_yaml_string = """
 - {type: standing, x: -10.0, y: 1.5, vx: 1.43, vy: 0.0}
 - {type: standing, x: -11.0, y: 0.5, vx: 0.7,  vy: 0.0}
 - {type: standing, x: -12.0, y: -2.0,vx: 0.7,  vy: 0.0}
 - {type: standing, x: 10.0,  y: -1.5, vx: -1.43,vy: 0.0}
 - {type: standing, x: 8.0,   y: 1.0, vx: -0.6, vy: 0.0}
 - {type: sitting,  x: 9.0,   y: -1.0, vx: -0.23, vy: 0.0}

"""
    
    
    # --- Human Simulation Parameters ---
    standing_human_mesh_path = 'package://sm_mppi_planner/models/Slampion/Slampion.dae'
    sitting_human_mesh_path = 'package://sm_mppi_planner/models/wheelchair/wheelchair.dae' 
    
    human_publisher_base_params = {
        'humans_yaml': humans_yaml_string,
        'human_radius': 0.3,
        'x_limits': [-(hallway_params['hallway_length']/2), (hallway_params['hallway_length']/2)],
        'y_limits': [-(hallway_params['hallway_width']/2), (hallway_params['hallway_width']/2)], 
        'startup_delay': startup_delay_seconds # <-- PASS DELAY TO HUMANS
    }

    # --- General Simulation Settings ---
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='true')
    launch_gazebo_arg = DeclareLaunchArgument('launch_gazebo', default_value='true')
        
    # ===================================================================================
    # ============== END OF SETTINGS - NO NEED TO EDIT BELOW THIS LINE ==================
    # ===================================================================================

    # --- Dynamic Generation of Planner Obstacles ---
    hw=hallway_params['hallway_width']/2.0; hl=hallway_params['hallway_length']/2.0
    wt=hallway_params['wall_thickness']/2.0; sb=planner_params['safety_boundary']
    hallway_walls_yaml_string=f"[[{-hl},{hw+wt+sb},{hl},{hw+wt+sb},{hl},{hw-wt-sb},{-hl},{hw-wt-sb}],[{-hl},{-hw-wt-sb},{hl},{-hw-wt-sb},{hl},{-hw+wt+sb},{-hl},{-hw+wt+sb}]]"
    
    # --- Gazebo Simulation ---
    tiago_simulation_group = GroupAction(condition=IfCondition(LaunchConfiguration('launch_gazebo')),actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(tiago_gazebo_pkg_share_dir,'launch','tiago_gazebo.launch.py')),launch_arguments={'is_public_sim':'True','world_name':'empty','use_sim_time':LaunchConfiguration('use_sim_time')}.items(),condition=IfCondition(PythonExpression([f"'{tiago_gazebo_pkg_share_dir}'!=''"])))])

    # --- RViz ---
    rviz_with_ld_preload_action=ExecuteProcess(cmd=['bash','-c',f"LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 rviz2 -d {rviz_config_file} --ros-args -p use_sim_time:=true"],output='screen',condition=IfCondition(LaunchConfiguration('launch_rviz')))

    # --- Your Nodes ---
    sm_mppi_planner_node = Node(
        package='sm_mppi_planner', executable='mppi_planner_node', name='sm_mppi_planner_node', output='screen',
        parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time'),'static_obstacles_yaml':hallway_walls_yaml_string,'static_cost_weight':planner_params['static_cost_weight']}]
    )

    goal_publisher_node = Node(
        package='sm_mppi_planner', executable='goal_publisher_node', name='goal_publisher_node', output='screen',
        parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time'),'goal_x':goal['x'],'goal_y':goal['y']}]
    )
    
    human_publisher_full_params = {
        'use_sim_time':LaunchConfiguration('use_sim_time'),'world_frame_id':'odom','standing_human_mesh_path':standing_human_mesh_path,'sitting_human_mesh_path':sitting_human_mesh_path,
    }
    human_publisher_full_params.update(human_publisher_base_params)
    human_publisher_node=Node(package='sm_mppi_planner',executable='fake_human_publisher',name='fake_human_publisher',output='screen',parameters=[human_publisher_full_params])

    hallway_publisher_node = Node(package='sm_mppi_planner',executable='hallway_publisher',name='hallway_publisher',output='screen',parameters=[hallway_params])

    # --- Assemble the Launch Description ---
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg); ld.add_action(launch_rviz_arg); ld.add_action(launch_gazebo_arg)
    ld.add_action(tiago_simulation_group)
    ld.add_action(rviz_with_ld_preload_action)
    ld.add_action(sm_mppi_planner_node)
    ld.add_action(goal_publisher_node)
    ld.add_action(human_publisher_node)
    ld.add_action(hallway_publisher_node)
    return ld