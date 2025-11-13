import os
from glob import glob
from setuptools import setup

package_name = 'sm_mppi_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install other directories as before
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        
        (os.path.join('share', package_name, 'models/walking_human'), [
            'models/walking_human/model.config',
            'models/walking_human/model.sdf'
        ]),
        (os.path.join('share', package_name, 'models/walking_human/meshes'), [
            'models/walking_human/meshes/walk.dae',
            'models/walking_human/meshes/moonwalk.dae'
        ]),


        (os.path.join('share', package_name, 'models/Slampion'), [
            'models/Slampion/model.config',
            'models/Slampion/model.sdf',
            'models/Slampion/Slampion.dae'
        ]),
        (os.path.join('share', package_name, 'models/Slampion/textures'), glob('models/Slampion/textures/*')),


        # This is your Wheelchair entry (structurally the same)
        (os.path.join('share', package_name, 'models/wheelchair'), [
            'models/wheelchair/model.config',
            'models/wheelchair/model.sdf',
            'models/wheelchair/wheelchair.dae'
        ]),
        (os.path.join('share', package_name, 'models/wheelchair/textures'), glob('models/wheelchair/textures/*')),

        (os.path.join('share', package_name, 'models/wall'), glob('models/wall/*')),

        (os.path.join('share', package_name, 'models/hallway_world'), glob('models/hallway_world/*')),
        (os.path.join('share', package_name, 'models/social_human'), glob('models/social_human/*')),
        (os.path.join('share', package_name, 'models/social_wheelchair'), glob('models/social_wheelchair/*')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'torch',
        'pytorch_mppi',
        'shapely',
        'rclpy',
        'geometry_msgs',
        'tf2_ros',
        'tf2_msgs',
        'visualization_msgs',
        'nav_msgs',
        'my_social_nav_interfaces',
        'gazebo_msgs',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Social Momentum MPPI Planner using PyTorch',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mppi_planner_node = sm_mppi_planner.mppi_planner_node:main',
            'goal_publisher_node = sm_mppi_planner.goal_publisher_node:main',
            # 'fake_human_publisher = sm_mppi_planner.fake_human_publisher:main', 
            'hallway_publisher = sm_mppi_planner.hallway_publisher:main',
            'gazebo_actor_relay = sm_mppi_planner.gazebo_actor_relay:main', 
            'metrics_logger = sm_mppi_planner.logger_node:main',
        ],
    },
)
