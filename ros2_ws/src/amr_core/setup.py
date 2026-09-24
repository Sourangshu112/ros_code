import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'amr_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
    ],
    install_requires=['setuptools', 'rclpy', 'nav_msgs', 'geometry_msgs', 'sensor_msgs', 'tf2_ros', 'fleet_interfaces'],
    zip_safe=True,
    maintainer='soura',
    maintainer_email='sourangshu098@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'peer_node = amr_core.peer_node:main'
        ],
    },
)
