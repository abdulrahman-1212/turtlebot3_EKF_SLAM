import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tb3_vio_ekf'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='EKF-fused VIO for TurtleBot3 simulation (rtabmap VO + robot_localization EKF).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_covariance_relay = tb3_vio_ekf.imu_covariance_relay:main',
            'vo_odom_relay = tb3_vio_ekf.vo_odom_relay:main',
        ],
    },
)
