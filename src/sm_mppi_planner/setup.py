from setuptools import setup
import os
from glob import glob

package_name = 'sm_mppi_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
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
        'nav_msgs'
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
        ],
    },
)
