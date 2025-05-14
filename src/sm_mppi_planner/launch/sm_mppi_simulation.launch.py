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
        print("[ERROR] Package 'tiago_gazebo' not found. TIAGo Gazebo will not be launched by this script if 'launch_gazebo' is true.")
        pass

    # --- Declare Launch Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='empty',
        description='Gazebo world name for tiago_gazebo (e.g., empty, pal_office)'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true', # Set to false if you prefer to always launch RViz manually
        description='Whether to launch RViz2'
    )

    launch_gazebo_arg = DeclareLaunchArgument(
        'launch_gazebo',
        default_value='true',
        description='Whether to launch Gazebo'
    )

    # --- TIAGo Gazebo Simulation ---
    tiago_simulation_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        actions=[
            LogInfo(
                condition=IfCondition(PythonExpression([f"'{tiago_gazebo_pkg_share_dir}' == ''"])),
                msg="tiago_gazebo package not found, Gazebo will not be launched by this script because 'launch_gazebo' is true but the package is missing."
            ),
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

    # --- RViz2 with LD_PRELOAD ---
    ld_preload_path = '/lib/x86_64-linux-gnu/libpthread.so.0'

    # Construct the command string for RViz2
    # Using LaunchConfiguration("use_sim_time").perform(None) might not work directly in f-string for ExecuteProcess
    # A common way is to build the command list more explicitly or use a wrapper.
    # For simplicity here, we'll assume use_sim_time will be 'true' for this example.
    # If you need it to be dynamic from the launch arg, it's a bit more involved with ExecuteProcess.
    # A simpler approach if use_sim_time is always true for this launch:
    rviz_command_str = f"LD_PRELOAD={ld_preload_path} ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true"
    # If you need to dynamically get the value of use_sim_time LaunchConfiguration:
    # This is more complex as ExecuteProcess cmd doesn't directly substitute LaunchConfigurations in the same way Node parameters do.
    # For now, hardcoding use_sim_time:=true as it's the default and typical for simulation.

    rviz_with_ld_preload_action = ExecuteProcess(
        cmd=['bash', '-c', rviz_command_str],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )
    # --- End RViz2 Node Definition ---

    # --- SM MPPI Planner Node ---
    sm_mppi_planner_node = Node(
        package='sm_mppi_planner',
        executable='mppi_planner_node',
        name='sm_mppi_planner_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    goal_publisher_node = Node(
        package='sm_mppi_planner',
        executable='goal_publisher_node',
        name='goal_publisher_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # --- Placeholder Static TF Broadcaster for ONE simulated human ---
    static_human_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_human_tf_publisher',
        output='screen',
        arguments=[
            '2.0', '0.5', '0.0', '0.0', '0.0', '0.0', '1.0', # x, y, z, qx, qy, qz, qw
            'map', # Parent frame
            'human_1'
        ],
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
    ld.add_action(static_human_tf_node)
    ld.add_action(goal_publisher_node)

    return ld
