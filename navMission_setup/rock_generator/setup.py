import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rock_generator'

def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/obs_data', glob('obs_data/*') if glob('obs_data/*') else []),
        ('share/' + package_name + '/Gen_worlds', glob('Gen_worlds/*') if glob('Gen_worlds/*') else []),
        ('share/' + package_name + '/heightmap_tools', glob('heightmap_tools/*') if glob('heightmap_tools/*') else []),
    ]
    
    # Recursively add rocks_ws directory files
    setup_dir = os.path.dirname(os.path.abspath(__file__))
    rocks_ws_dir = os.path.join(setup_dir, 'rocks_ws')
    if os.path.exists(rocks_ws_dir):
        for root, dirs, files in os.walk(rocks_ws_dir):
            if files:
                rel_dir = os.path.relpath(root, rocks_ws_dir)
                dest_dir = os.path.join('share', package_name, 'rocks_ws', rel_dir if rel_dir != '.' else '')
                # Make paths relative to setup_dir so colcon doesn't complain about absolute paths
                rel_root = os.path.relpath(root, setup_dir)
                file_paths = [os.path.join(rel_root, f) for f in files]
                data_files.append((dest_dir, file_paths))

    return data_files

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=['setuptools', 'numpy', 'pillow'],
    zip_safe=True,
    maintainer='Saif / ROAR Simulation Team',
    maintainer_email='saif@roar.edu',
    description='Obstacle data generator, world generator, and rock spawner for ROAR simulation worlds.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_obs = rock_generator.generator:main',
            'generate_world = rock_generator.world_generator:main',
            'spawn_rocks = rock_generator.spawner:main',
            'rock_generator = rock_generator.main:main',
        ],
    },
)
