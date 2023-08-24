from setuptools import find_packages
from setuptools import setup

setup(
    name='irobot_create_common_bringup',
    version='2.0.0',
    packages=find_packages(
        include=('irobot_create_common_bringup', 'irobot_create_common_bringup.*')),
)
