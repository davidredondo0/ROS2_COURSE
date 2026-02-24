from setuptools import find_packages
from setuptools import setup

setup(
    name='as2_motion_reference_handlers',
    version='1.1.3',
    packages=find_packages(
        include=('as2_motion_reference_handlers', 'as2_motion_reference_handlers.*')),
)
