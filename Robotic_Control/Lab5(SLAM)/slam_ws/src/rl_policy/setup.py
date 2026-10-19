import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rl_policy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('models/*.npz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wpenhall',
    maintainer_email='wpenhall@uri.edu',
    description='Drives an F1TENTH car (real or AutoDRIVE) with the LiDAR policy trained in CarRacing',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'policy_node = rl_policy.policy_node:main'
        ],
    },
)
