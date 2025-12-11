import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    RegisterEventHandler, 
    EmitEvent,
    LogInfo
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- Arguments ---
    # 1. Path to the bag file you want to reprocess
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        description='Full path to the input ROS bag (folder or mcap file)'
    )
    
    # 2. Name of the output parquet file
    output_filename_arg = DeclareLaunchArgument(
        'output_filename',
        default_value='recovered_data.parquet',
        description='Name of the output Parquet file'
    )

    # --- Configurations ---
    bag_path = LaunchConfiguration('bag_path')
    output_filename = LaunchConfiguration('output_filename')

    # --- 1. The Logger Node ---
    # We set use_sim_time to True so it listens to the bag's clock
    metrics_logger_node = Node(
        package='sm_mppi_planner',
        executable='metrics_logger',
        name='metrics_logger',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'output_filename': output_filename},
            {'slop_seconds': 0.15}
        ]
    )

    # --- 2. The Bag Player ---
    # --clock: Publishes /clock so the node knows the simulation time
    # -r 1.0: Plays at real-time speed (adjust if your computer is slow)
    bag_play_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', 
            bag_path, 
            '--clock',
            '-r', '1.0' 
        ],
        output='screen'
    )

    # --- 3. The Auto-Shutdown Handler ---
    # This ensures that when the bag finishes, we send a SIGINT/Shutdown
    # signal to the logger node so it can write the file.
    shutdown_on_finish = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=bag_play_process,
            on_exit=[
                LogInfo(msg='Bag playback finished. Shutting down to save Parquet...'),
                EmitEvent(event=Shutdown())
            ]
        )
    )

    return LaunchDescription([
        bag_path_arg,
        output_filename_arg,
        metrics_logger_node,
        bag_play_process,
        shutdown_on_finish
    ])