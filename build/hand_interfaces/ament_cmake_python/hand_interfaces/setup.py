from setuptools import find_packages
from setuptools import setup

setup(
    name='hand_interfaces',
    version='0.0.1',
    packages=find_packages(
        include=('hand_interfaces', 'hand_interfaces.*')),
)
