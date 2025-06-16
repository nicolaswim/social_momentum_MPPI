from setuptools import find_packages
from setuptools import setup

setup(
    name='my_social_nav_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('my_social_nav_interfaces', 'my_social_nav_interfaces.*')),
)
