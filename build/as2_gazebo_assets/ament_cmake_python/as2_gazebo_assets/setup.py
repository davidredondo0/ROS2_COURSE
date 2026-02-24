from setuptools import find_packages
from setuptools import setup

setup(
    name='as2_gazebo_assets',
    version='1.1.3',
    packages=find_packages(
        include=('as2_gazebo_assets', 'as2_gazebo_assets.*')),
)
