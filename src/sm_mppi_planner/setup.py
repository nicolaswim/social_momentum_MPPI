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
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        
        # --- CORRECTED & EXPLICIT MODEL INSTALLATION ---
        # Install the model description files
        (os.path.join('share', package_name, 'models/walking_human'), [
            'models/walking_human/model.config',
            'models/walking_human/model.sdf'
        ]),
        # Install the mesh files into their own subdirectory
        (os.path.join('share', package_name, 'models/walking_human/meshes'), [
            'models/walking_human/meshes/walk.dae',
            'models/walking_human/meshes/moonwalk.dae'
        ]),
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
            'fake_human_publisher = sm_mppi_planner.fake_human_publisher:main',
        ],
    },
)