from setuptools import find_packages
from setuptools import setup

setup(
    name='pendulum2_msgs',
    version='0.0.1',
    packages=find_packages(
        include=('pendulum2_msgs', 'pendulum2_msgs.*')),
)
