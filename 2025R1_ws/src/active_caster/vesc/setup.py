from setuptools import find_packages, setup

package_name = 'vesc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'setuptools',
        'pyserial>=3.0,<4.0',
        'pyvesc>=1.0.5',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    maintainer='steven',
    maintainer_email='yanczhang8@gmail.com',
    description='VESC control for active wheel',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'active_wheel = vesc.active_wheel:main',
        ],
    },
)
