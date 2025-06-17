import os
from glob import glob
from setuptools import setup

package_name = 'r1_launch'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Install package.xml
        (os.path.join('share', package_name), ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install scripts
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
        # Install resource index
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), 
         [os.path.join('resource', package_name)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Steven Zhang',
    maintainer_email='s1153766@s.eduhk.hk',
    description='Launch files for R1 robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add executable scripts here if needed, e.g., 'script_name = r1_launch.module:main'
        ],
    },
)