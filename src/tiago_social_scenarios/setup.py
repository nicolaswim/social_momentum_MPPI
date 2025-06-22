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
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install map files
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        # Install all models
        (os.path.join('share', package_name, 'models/hallway_world'), glob('models/hallway_world/*')),
        (os.path.join('share', package_name, 'models/social_human'), glob('models/social_human/*')),
        (os.path.join('share', package_name, 'models/social_wheelchair'), glob('models/social_wheelchair/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wim',
    maintainer_email='wim@todo.todo',
    description='Scenario package for TIAGo social navigation validation.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # We will add the new Gazebo human controller here later
        ],
    },
)