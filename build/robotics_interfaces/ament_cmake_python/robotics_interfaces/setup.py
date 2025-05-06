from setuptools import find_packages
from setuptools import setup

setup(
    name='robotics_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('robotics_interfaces', 'robotics_interfaces.*')),
)
