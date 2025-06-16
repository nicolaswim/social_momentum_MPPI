import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, LogInfo, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true')

    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='empty', description='Gazebo world name')

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='true', description='Whether to launch RViz2')

    launch_gazebo_arg = DeclareLaunchArgument(
        'launch_gazebo', default_value='true', description='Whether to launch Gazebo')

    # Adjusting config values first
    adjust_config_action = TimerAction(
        period=1.0,  # Delay before applying config changes
        actions=[
            LogInfo(msg="Adjusting config parameters before launching nodes...")
        ]
    )

    # Launch actions
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
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Fake humans in random mode (with 5 agents)
    human_publisher_node = Node(
        package='sm_mppi_planner',
        executable='fake_human_publisher',
        name='fake_human_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'scenario_mode': 'random'},
            {'num_random_humans': 5},
            {'x_limits': [-5.0, 5.0]},
            {'y_limits': [-5.0, 5.0]},
            {'world_frame_id': 'odom'},
            {'publish_frequency': 10.0},
            {'human_max_speed': 0.3},
            {'human_min_speed': 0.1},
            {'initial_delay_sec': 1.5},
            {'humans_topic': '/humans'},
            {'markers_topic': '/human_markers'}
        ]
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(world_name_arg)
    ld.add_action(launch_rviz_arg)
    ld.add_action(launch_gazebo_arg)

    ld.add_action(adjust_config_action)  # Adjust config values first
    ld.add_action(sm_mppi_planner_node)
    ld.add_action(goal_publisher_node)
    ld.add_action(human_publisher_node)

    return ld
