from setuptools import find_packages
from setuptools import setup

setup(
    name='as2_core',
    version='1.1.3',
    packages=find_packages(
        include=('as2_core', 'as2_core.*')),
)
