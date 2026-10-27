from setuptools import setup
import os
from glob import glob

package_name = 'roar_slam_testing'

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
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Ashraf',
    maintainer_email='mohamed.ashraf.eng2002@example.com',
    description='SLAM module test harness (obstacle simulator + evaluator + visualizer).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'obstacle_simulator = roar_slam_testing.obstacle_simulator:main',
            'slam_evaluator = roar_slam_testing.slam_evaluator:main',
            'slam_visualizer = roar_slam_testing.slam_visualizer:main',
        ],
    },
)
