#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from setuptools import setup
import os
from glob import glob

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'pyserial>=3.0,<4.0',
        'pyvesc>=1.0.5',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    zip_safe=True,
    maintainer='Steven_Zhang',
    maintainer_email='s1153766@eduhk.hk',
    description='ROS2 package for robot navigation, including active caster control.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = navigation.navigation_node:main',
            'active_caster_ctrl_node = navigation.active_caster_ctrl_node:main',
            'omni_wheel_speed_node = navigation.omni_wheel_speed_node:main'
        ],
    },
)
