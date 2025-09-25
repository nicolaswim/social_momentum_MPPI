import os
from glob import glob
from setuptools import setup

package_name = 'tiago_social_scenarios'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.*')),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*'))),
        (os.path.join('share', package_name, 'models/hallway_world'), glob('models/hallway_world/*')),
        (os.path.join('share', package_name, 'models/social_human'), glob('models/social_human/*')),
        (os.path.join('share', package_name, 'models/social_wheelchair'), glob('models/social_wheelchair/*')),
        # Add this line inside the data_files list in tiago_social_scenarios/setup.py
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wim',
    maintainer_email='wim@todo.todo',
    description='Scenario package for TIAGo social navigation validation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_human_controller = tiago_social_scenarios.gazebo_human_controller:main',
        ],
    },
)