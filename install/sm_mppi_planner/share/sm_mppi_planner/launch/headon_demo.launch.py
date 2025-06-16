import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, LogInfo, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    tiago_gazebo_pkg_share_dir = ""
    try:
        tiago_gazebo_pkg_share_dir = get_package_share_directory('tiago_gazebo')
    except PackageNotFoundError:
        print("[ERROR] Package 'tiago_gazebo' not found.")

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value='empty',
        description='Gazebo world name')

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='true',
        description='Whether to launch RViz2')

    launch_gazebo_arg = DeclareLaunchArgument(
        'launch_gazebo', default_value='true',
        description='Whether to launch Gazebo')

    tiago_simulation_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        actions=[
            LogInfo(
                condition=IfCondition(PythonExpression([f"'{tiago_gazebo_pkg_share_dir}' == ''"])),
                msg="tiago_gazebo package not found. Gazebo won't launch.") ,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tiago_gazebo_pkg_share_dir, 'launch', 'tiago_gazebo.launch.py')
                ),
                launch_arguments={
                    'is_public_sim': 'True',
                    'world_name': LaunchConfiguration('world_name'),
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items(),
                condition=IfCondition(PythonExpression([f"'{tiago_gazebo_pkg_share_dir}' != ''"]))
            )
        ]
    )

    ld_preload_path = '/lib/x86_64-linux-gnu/libpthread.so.0'
    rviz_command_str = f"LD_PRELOAD={ld_preload_path} ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true"

    rviz_with_ld_preload_action = ExecuteProcess(
        cmd=['bash', '-c', rviz_command_str],
        output='screen',

        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

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

    # New: Head-on human publisher
    human_publisher_node = Node(
        package='sm_mppi_planner',
        executable='fake_human_publisher',
        name='fake_human_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'scenario_mode': 'head_on'},
            {'num_random_humans': 0},
            {'x_limits': [-5.0, 5.0]},
            {'y_limits': [-5.0, 5.0]},
            {'world_frame_id': 'odom'},
            {'publish_frequency': 10.0},
            {'human_max_speed': 0.3},
            {'human_min_speed': 0.3},
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

    ld.add_action(tiago_simulation_group)
    ld.add_action(rviz_with_ld_preload_action)
    ld.add_action(sm_mppi_planner_node)
    ld.add_action(goal_publisher_node)
    ld.add_action(human_publisher_node)

    return ld
