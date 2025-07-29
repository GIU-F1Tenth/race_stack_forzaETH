from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'horizon_mapper'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include data files
        ('share/' + package_name + '/data', [
            'horizon_mapper/optimal_trajectory.csv',
            'horizon_mapper/ref_trajectory.csv'
        ]),
        # Include launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Include config files if any
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohammedazab',
    maintainer_email='mohammedazab@example.com',
    description='Horizon Mapper - Trajectory processing and publishing package for F1TENTH MPC controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'horizon_mapper_node = horizon_mapper.horizon_mapper_node:main',
        ],
    },
)
