from setuptools import setup
import os
from glob import glob

package_name = 'joystick_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files if you have them
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS 2 package to publish joystick inputs using evdev.',
    license='Apache License 2.0', # Or your preferred license
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_node = joystick_driver.joystick_publisher_node:main',
        ],
    },
)

