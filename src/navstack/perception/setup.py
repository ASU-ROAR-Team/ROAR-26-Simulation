from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'perception_module'

setup(
    name=package_name,
    version='0.0.0',
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
    maintainer='carlos',
    maintainer_email='c.argueta@s1s2.ai',
    description='Modular 3D Perception Pipeline',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_cluster_node = perception_module.lidar_cluster_node:main',
            'camera_detector_node = perception_module.camera_detector_node:main',
            'frustum_fusion_node = perception_module.frustum_fusion_node:main',
            'bev_visualizer_node = perception_module.bev_visualizer_node:main',
            'obstacle_publisher_node = perception_module.obstacle_publisher_node:main',
            'aruco_detector_node = perception_module.aruco_detector_node:main',
        ],
    },
)
