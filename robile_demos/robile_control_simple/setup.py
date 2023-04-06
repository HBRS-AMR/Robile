from setuptools import setup

package_name = 'robile_control_simple'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='melody',
    maintainer_email='kishansawant96@gmail.com',
    description='Move robot by publishing to cmd_vel topic',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circular_motion_publisher = robile_control_simple.robile_control_simple:main',
        ],
    },
)
