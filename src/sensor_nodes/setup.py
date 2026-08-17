from setuptools import setup

package_name = 'sensor_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'gps_node = sensor_nodes.gps_node:main',
            'imu_node = sensor_nodes.imu_node:main',
            'localshift = sensor_nodes.localshift:main',
        ],
    },
)
