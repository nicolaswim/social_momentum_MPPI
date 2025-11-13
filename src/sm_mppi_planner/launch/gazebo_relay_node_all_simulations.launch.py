import os
import datetime
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    GroupAction, 
    ExecuteProcess, 
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # --- Package Directories ---
    pkg_share_dir = get_package_share_directory('sm_mppi_planner')
    tiago_gazebo_pkg_share_dir = ""
    try:
        tiago_gazebo_pkg_share_dir = get_package_share_directory('tiago_gazebo')
    except PackageNotFoundError:
        print("[ERROR] Package 'tiago_gazebo' not found. Gazebo will not be launched.")

    # --- RViz Configuration ---
    rviz_config_file = os.path.join(pkg_share_dir, 'rviz', 'hallway_test.rviz')

    # --- Hallway Configuration ---
    hallway_params = {
        'hallway_length': 20.0,
        'hallway_width': 5.0,
        'wall_thickness': 0.1,
        'wall_height': 2.5,
        'wall_mesh_path': 'package://sm_mppi_planner/models/wall/wall.dae'
    }

    # --- Simulation Startup Delay ---
    simulation_startup_delay = 21.0

    # --- Navigation Goal ---
    goal = {'x': 9.0, 'y': 0.0}

    # --- Planner Configuration ---
    planner_params = {
        'static_cost_weight': 25.0,
        'safety_boundary': 0.2
    }

    # --- General Simulation Settings ---
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='true')
    launch_gazebo_arg = DeclareLaunchArgument('launch_gazebo', default_value='true')

    # --- Launch Argument for Scenario ID ---
    scenario_id_arg = DeclareLaunchArgument(
        'scenario_id',
        default_value='1',
        description='The ID of the scenario to load (1-5).'
    )
    # Get the value of the new argument
    scenario_id = LaunchConfiguration('scenario_id')

    # --- Timestamp and output name ---
    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs('rosbags/raw', exist_ok=True)

    # Bag name: rosbags/raw/scenario_backup_<id>_<ts>
    output_bag_file = [
        TextSubstitution(text='rosbags/raw/scenario_backup_'),
        scenario_id,
        TextSubstitution(text='_'),
        TextSubstitution(text=timestamp_str)
    ]

    # Topics to record
    topics_to_record = [
        '/mobile_base_controller/odom',
        '/joint_states',
        '/clock',
        '/tf',
        '/tf_static',
        '/model_states',
        '/goal_pose',
        '/social_nav/humans',
        '/mobile_base_controller/cmd_vel_unstamped'
    ]

    # Compressed rosbag2 record
    rosbag_record_action = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--compression-mode', 'message',
            '--compression-format', 'zstd',
            '-o', output_bag_file
        ] + topics_to_record,
        output='screen',
        emulate_tty=True,
        shell=False
    )

    
    # --- FIX: Reverted JoinSubstitution back to a simple list ---
    output_parquet_file = [
        TextSubstitution(text='rosbags/raw/metrics_data_'),
        scenario_id,
        TextSubstitution(text='_'),
        TextSubstitution(text=timestamp_str),
        TextSubstitution(text='.parquet')
    ]

    metrics_logger_node = Node(
        package='sm_mppi_planner',
        executable='metrics_logger',
        name='metrics_logger',
        output='screen',
        parameters=[
            {'output_filename': output_parquet_file}, # Pass the list directly
            {'slop_seconds': 0.15}, 
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # --- Dynamic Generation of Planner Obstacles ---
    hw=hallway_params['hallway_width']/2.0; hl=hallway_params['hallway_length']/2.0
    wt=hallway_params['wall_thickness']/2.0; sb=planner_params['safety_boundary']
    hallway_walls_yaml_string=f"[[{-hl},{hw+wt+sb},{hl},{hw+wt+sb},{hl},{hw-wt-sb},{-hl},{hw-wt-sb}],[{-hl},{-hw-wt-sb},{hl},{-hw-wt-sb},{hl},{-hw+wt+sb},{-hl},{-hw+wt+sb}]]"
    
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
                    # --- MODIFIED: Build the world_name from the argument ---
                    'world_name': [TextSubstitution(text='scenario_'), scenario_id],
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items(),
                condition=IfCondition(PythonExpression([f"'{tiago_gazebo_pkg_share_dir}'!=''"]))
            )
        ]
    )

    # --- RViz ---
    rviz_with_ld_preload_action = ExecuteProcess(
        cmd=['bash', '-c', f"LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 rviz2 -d {rviz_config_file} --ros-args -p use_sim_time:=true"],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    # Define agent counts for each scenario
    scenario_agent_counts = {
        '1': 5,  # 4 humans + 1 wheelchair [cite: 93, 100, 107, 114, 121]
        '2': 3,  # 2 humans + 1 wheelchair [cite: 10, 14, 18]
        '3': 4,  # 2 humans + 2 wheelchairs [cite: 31, 33, 35, 39]
        '4': 2,  # 2 humans [cite: 136, 141]
        '5': 6,  # 6 humans [cite: 53, 58, 63, 68, 73, 78]
    }

    # Create a substitution that looks up the agent count from the dictionary
    active_agents_sub = PythonExpression([
        str(scenario_agent_counts),
        "['",
        scenario_id,
        "']"
    ])
    # --- Define Your Nodes ---
    sm_mppi_planner_node = Node(
            package='sm_mppi_planner', 
            executable='mppi_planner_node', 
            name='sm_mppi_planner_node', 
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'static_obstacles_yaml': hallway_walls_yaml_string},
                {'static_cost_weight': planner_params['static_cost_weight']},
                {'human_topic': '/social_nav/humans'},
                {'startup_delay': 1.0},
                {'active_agents': active_agents_sub}
            ]
    )

    goal_publisher_node = Node(
        package='sm_mppi_planner', executable='goal_publisher_node', name='goal_publisher_node', output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'goal_x': goal['x']},
            {'goal_y': goal['y']}
        ]
    )
    
    gazebo_actor_relay_node = Node(
        package='sm_mppi_planner',
        executable='gazebo_actor_relay',
        name='gazebo_actor_relay',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'planner_frame': 'odom'}, 
            {'publish_rate_hz': 30.0}
        ]
    )

    hallway_publisher_node = Node(
        package='sm_mppi_planner',
        executable='hallway_publisher',
        name='hallway_publisher',
        output='screen',
        parameters=[
            hallway_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')} 
        ]
    )

    # --- Use TimerAction to delay the navigation stack ---
    delayed_nav_stack = TimerAction(
        period=simulation_startup_delay,
        actions=[
            sm_mppi_planner_node,
            goal_publisher_node,
            gazebo_actor_relay_node,
            rosbag_record_action,
            metrics_logger_node
        ]
    )

    # --- Assemble the Launch Description ---
    ld = LaunchDescription()
    
    # --- ADDED: Add the new argument to the launch description ---
    ld.add_action(scenario_id_arg)

    ld.add_action(use_sim_time_arg)
    ld.add_action(launch_rviz_arg)
    ld.add_action(launch_gazebo_arg)
    
    # Launch Gazebo and RViz immediately
    ld.add_action(tiago_simulation_group)
    ld.add_action(rviz_with_ld_preload_action)
    ld.add_action(hallway_publisher_node) # This can also start immediately

    # Launch the rest of the stack after the 20s delay
    ld.add_action(delayed_nav_stack)
    
    return ld