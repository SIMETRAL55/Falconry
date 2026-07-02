import os
from glob import glob

from setuptools import setup

package_name = 'drone_follow'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wasim Shaikh',
    maintainer_email='wasim.shaikh@lexxpluss.com',
    description='Detect-select-follow: perception and 20 Hz offboard follower for PX4.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = drone_follow.perception_node:main',
            'follower_node = drone_follow.follower_node:main',
        ],
    },
)
