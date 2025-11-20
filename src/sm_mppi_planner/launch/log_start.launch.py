import os
import datetime
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction # Still needed, but we won't use it to delay the main actions
)
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # --- Arguments ---
    scenario_id_arg = DeclareLaunchArgument(
        'scenario_id',
        default_value='1',
        description='The ID of the scenario being run for naming the log files.'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock.'
    )

    # Get the value of the arguments
    scenario_id = LaunchConfiguration('scenario_id')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- Timestamp and output name ---
    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    # --- ROS Bag Configuration ---
    output_bag_file = [
        TextSubstitution(text='rosbags/raw/scenario_backup_'),
        scenario_id,
        TextSubstitution(text='_'),
        TextSubstitution(text=timestamp_str)
    ]

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

    # Compressed rosbag2 record action (Starts immediately)
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

    # --- Metrics Logger Node Configuration ---
    output_parquet_file = [
        TextSubstitution(text='rosbags/raw/metrics_data_'),
        scenario_id,
        TextSubstitution(text='_'),
        TextSubstitution(text=timestamp_str),
        TextSubstitution(text='.parquet')
    ]

    # Metrics Logger Node (Starts immediately)
    metrics_logger_node = Node(
        package='sm_mppi_planner',
        executable='metrics_logger',
        name='metrics_logger',
        output='screen',
        parameters=[
            {'output_filename': output_parquet_file},
            {'slop_seconds': 0.15},
            {'use_sim_time': use_sim_time}
        ]
    )

    # --- Assemble the Launch Description ---
    ld = LaunchDescription()

    ld.add_action(scenario_id_arg)
    ld.add_action(use_sim_time_arg)

    # Launch the logger and rosbag immediately
    ld.add_action(rosbag_record_action)
    ld.add_action(metrics_logger_node)

    return ld