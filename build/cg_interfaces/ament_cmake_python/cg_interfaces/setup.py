from setuptools import find_packages
from setuptools import setup

setup(
    name='cg_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('cg_interfaces', 'cg_interfaces.*')),
)
