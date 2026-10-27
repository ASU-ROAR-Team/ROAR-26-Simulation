from setuptools import setup, find_packages  # <--- Note the extra import
import os
from glob import glob

package_name = 'adaptive_pure_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    # CHANGED: Auto-find packages instead of hardcoding
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Copy Launch Files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        
        # Copy Config Files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),

        # Copy RViz Files
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),

        # Copy URDF Files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Amr Kassab',
    maintainer_email='amrkassab2005@gmail.com',
    description='Adaptive Pure Pursuit controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Ensure this matches your filename "AdaptiveLd.py" exactly (Case Sensitive!)
            'adaptiveLd = adaptive_pure_pursuit.AdaptiveLd:main',
            'adaptiveLd_fixed = adaptive_pure_pursuit.AdaptiveLd_fixed:main',
            'test_orchestrator = adaptive_pure_pursuit.test_orchestrator:main',
            'control_monitor = adaptive_pure_pursuit.control_monitor:main',
            'rviz_visualizer = adaptive_pure_pursuit.rviz_visualizer:main',
            'path_test = adaptive_pure_pursuit.pathTest:main',
            'path_comprehensive = adaptive_pure_pursuit.comprehensive_path_gen:main',
            'validator_complete = adaptive_pure_pursuit.validator_complete:main',
            'validation_node = adaptive_pure_pursuit.validation_node:main',
            'visual_control = adaptive_pure_pursuit.visual_control:main',
            'nav2_bench_bridge = adaptive_pure_pursuit.nav2_bench_bridge:main',
            'costmap_publisher = adaptive_pure_pursuit.costmap_publisher:main',
            'obstacle_generator = adaptive_pure_pursuit.obstacle_generator:main',
        ],
    },
)