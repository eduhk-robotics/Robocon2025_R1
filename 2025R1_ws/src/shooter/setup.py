from setuptools import setup
import os
from glob import glob

package_name = 'shooter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ?? launch ????????
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Steven Zhang',
    maintainer_email='s1153766@s.eduhk.hk',
    description='ROS 2 package for shooter',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shooter_vesc_node = shooter.shooter_vesc_node:main',
            'shooter_control_node = shooter.shooter_control_node:main',
            'shooter_damiao_node = shooter.shooter_damiao_node:main',
        ],
    },
)
