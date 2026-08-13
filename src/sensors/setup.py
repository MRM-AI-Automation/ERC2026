from setuptools import find_packages, setup

package_name = 'sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        'pynmea2',
    ],
    zip_safe=True,
    maintainer='mrmnavjet',
    maintainer_email='',
    description='GPS and IMU sensor ROS 2 nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_node = sensors.gps_node:main',
            'imu_node = sensors.imu_node:main',
        ],
    },
)
