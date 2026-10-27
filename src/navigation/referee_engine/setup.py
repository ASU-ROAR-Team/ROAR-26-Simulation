import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'referee_engine'

# Recursively find all files in the inputs directory and map their relative paths
input_files = []
for root, dirs, files in os.walk('inputs'):
    for file in files:
        if file.endswith('.npy'):
            full_path = os.path.join(root, file)
            target_path = os.path.join('share', package_name, root)
            input_files.append((target_path, [full_path]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ] + input_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='misara',
    maintainer_email='misaraaboushouier@gmail.com',
    description='Automated Referee scoring engine for ERC simulation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'referee_core_node = referee_engine.referee_core_node:main',
            'report_generator_node = referee_engine.report_generator_node:main',
            'collision_monitor_node = referee_engine.collision_monitor_node:main',
        ],
    },
)