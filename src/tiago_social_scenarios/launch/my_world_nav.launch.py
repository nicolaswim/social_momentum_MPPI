from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    overrides = RewrittenYaml(
        source_file='/home/wim/Documents/social_momentum_venv/social_momentum_MPPI/src/tiago_social_scenarios/config/nav2_override.yaml',
        root_key=None,
        param_rewrites={},      # required
        convert_types=True
    )

    tiago_launch = PythonLaunchDescriptionSource(
        [get_package_share_directory('tiago_gazebo'), '/launch/tiago_gazebo.launch.py']
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            tiago_launch,
            launch_arguments={
                'navigation': 'True',
                'is_public_sim': 'True',
                'world_name': 'my_world',
                'params_file': overrides,        # if this arg isn’t supported, try the next lines one at a time:
                # 'nav2_params_file': overrides,
                # 'default_nav2_params_path': overrides,
            }.items()
        )
    ])
