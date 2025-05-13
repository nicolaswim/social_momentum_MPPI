from setuptools import setup
import os
from glob import glob

package_name = 'sm_mppi_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name], # This tells setuptools to look for an inner folder named 'sm_mppi_planner' for Python modules
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]), # The marker file in your resource/ folder
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # If you had other resource files like RViz configs to install:
        # (os.path.join('share', package_name, 'resource'), glob('resource/*.rviz')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'torch',      # Ensure torch is installed in your Python env (pip install torch)
        'pytorch_mppi', # If this is a pip-installable package, uncomment. Otherwise, ensure it's accessible.
        'shapely',    # Ensure shapely is installed (pip install shapely)
        'rclpy',
        'geometry_msgs',
        'tf2_ros',
        'tf2_msgs',
        'visualization_msgs',
        'nav_msgs'
    ],
    zip_safe=True,
    maintainer='Your Name', # Replace with your name
    maintainer_email='your_email@example.com', # Replace with your email
    description='Social Momentum MPPI Planner using PyTorch',
    license='Apache License 2.0', # Or your preferred license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mppi_planner_node = sm_mppi_planner.mppi_planner_node:main',
        ],
    },
)