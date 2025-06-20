from setuptools import setup

package_name = 'bounce'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[
        'bounce.bounce_control_node',
        'bounce.bounce_relays_node',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Steven Zhang',
    maintainer_email='s1153766@s.eduhk.hk',
    description='ROS package for bounce control with joystick input',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bounce_control_node = bounce.bounce_control_node:main',
            'bounce_relays_node = bounce.bounce_relays_node:main',
        ],
    },
)
