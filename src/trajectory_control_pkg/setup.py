from setuptools import setup
from glob import glob
import os

package_name = 'trajectory_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # 🔥 THIS installs launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    entry_points={
        'console_scripts': [
            'path_smoother_node = trajectory_control_pkg.path_smoother_node:main',
            'trajectory_generator_node = trajectory_control_pkg.trajectory_generator_node:main',
            'trajectory_tracker_node = trajectory_control_pkg.trajectory_tracker_node:main',
        ],
    },
)

