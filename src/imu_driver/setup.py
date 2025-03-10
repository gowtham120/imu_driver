import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'imu_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'imu_driver'), glob('launch/*imu_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gowtham',
    maintainer_email='parasuraman.g@northeastern.edu',
    description='package contains the ROS2 driver node for IMU sensor',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'IMU_ROS2driver_g = imu_driver.driver:main',
        ],
    },
)
