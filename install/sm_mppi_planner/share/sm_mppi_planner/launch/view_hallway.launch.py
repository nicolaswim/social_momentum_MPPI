import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    """
    Minimal launch file to visualize the hallway markers in RViz.
    """
    
    # Get the share directory for your package to find the RViz config file
    pkg_share_dir = get_package_share_directory('sm_mppi_planner')

    # Path to the RViz configuration file - CORRECTED THE FILENAME
    rviz_config_file = os.path.join(pkg_share_dir, 'rviz', 'hallway.rviz') 

    # ===================================================================================
    # =============================== LAUNCH ACTIONS ====================================
    # ===================================================================================

    # Argument to decide whether to launch RViz
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    # Node for your new hallway publisher
    hallway_publisher_node = Node(
        package='sm_mppi_planner',
        executable='hallway_publisher',
        name='hallway_publisher',
        output='screen'
    )

    # Corrected RViz launch action using ExecuteProcess
    rviz_node = ExecuteProcess(
        cmd=['bash', '-c', f'LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0 rviz2 -d {rviz_config_file}'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    # --- Assemble the Launch Description ---
    ld = LaunchDescription()
    
    ld.add_action(launch_rviz_arg)
    ld.add_action(hallway_publisher_node)
    ld.add_action(rviz_node)

    return ld