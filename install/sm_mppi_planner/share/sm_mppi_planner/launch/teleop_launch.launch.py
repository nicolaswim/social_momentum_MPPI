import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, LogInfo, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    try:
        tiago_gazebo_pkg_share_dir = get_package_share_directory('tiago_gazebo')
    except PackageNotFoundError:
        tiago_gazebo_pkg_share_dir = ''

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    world_name_arg = DeclareLaunchArgument('world_name', default_value='empty')
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='true')
    launch_gazebo_arg = DeclareLaunchArgument('launch_gazebo', default_value='true')

    tiago_simulation_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        actions=[
            LogInfo(
                condition=IfCondition(PythonExpression([f"'{tiago_gazebo_pkg_share_dir}' == ''"])),
                msg="tiago_gazebo package not found. Gazebo won't launch."),
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

    # Note: LD_PRELOAD is a workaround for some rviz/OGRE issues.
    # Ensure the path to libpthread.so.0 is correct for your system or remove if not needed.
    ld_preload_path = '/lib/x86_64-linux-gnu/libpthread.so.0' # Common path, adjust if necessary
    # Check if the file exists, otherwise don't preload
    # This is a simplified check; a more robust check might be desired in a production script
    rviz_cmd = ['ros2', 'run', 'rviz2', 'rviz2', '--ros-args', '-p', 'use_sim_time:=true']
    if os.path.exists(ld_preload_path):
        rviz_cmd_str = f"LD_PRELOAD={ld_preload_path} {' '.join(rviz_cmd)}"
    else:
        rviz_cmd_str = ' '.join(rviz_cmd)
        # Optionally log a warning if the preload library is not found
        # LogInfo(msg=f"LD_PRELOAD library {ld_preload_path} not found. Running RViz without it.")


    rviz_with_ld_preload_action = ExecuteProcess(
        cmd=['bash', '-c', rviz_cmd_str],
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

    # Modified human_publisher_node for teleop mode
    human_publisher_node_teleop = Node(
        package='sm_mppi_planner',
        executable='fake_human_publisher',
        name='fake_human_publisher_teleop', # Node name slightly changed for clarity
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'scenario_mode': 'teleop'},  # <<< Changed to teleop mode
            {'num_random_humans': 0},      # Not used in teleop, but kept for consistency
            {'x_limits': [-5.0, 5.0]},     # y_limits used for initial teleop human y-pos
            {'y_limits': [0.0, 0.0]},      # Initial teleop human will start at y=0.0
            {'world_frame_id': 'odom'},
            {'publish_frequency': 10.0},
            {'human_max_speed': 0.3},      # Not directly used by teleop human
            {'human_min_speed': 0.3},      # Not directly used by teleop human
            {'initial_delay_sec': 1.5},
            {'humans_topic': '/humans'},
            {'markers_topic': '/human_markers'}
            # 'teleop_cmd_topic' is defaulted to '/human_teleop_cmd_vel' in your node,
            # which matches the keyboard teleop output.
        ]
    )

    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='human_teleop_keyboard', # This name is fine
        output='screen',
        remappings=[('/cmd_vel', '/human_teleop_cmd_vel')], # Publishes to the topic FakeHumanPublisher listens to
        prefix='xterm -e', # Launches the keyboard teleop in a new terminal
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
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
    ld.add_action(human_publisher_node_teleop) # Added the modified node
    ld.add_action(teleop_keyboard_node)

    return ld