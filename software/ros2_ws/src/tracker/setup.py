from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tracker'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Ground-based radar and camera drone tracker',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'radar_driver     = tracker.radar_driver:main',
            'camera_tracker   = tracker.camera_tracker:main',
            'ekf_controller   = tracker.ekf_controller:main',
            'gimbal_controller = tracker.gimbal_controller:main',
        ],
    },
)
