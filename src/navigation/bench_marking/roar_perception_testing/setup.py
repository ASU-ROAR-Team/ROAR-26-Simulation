from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'roar_perception_testing'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    maintainer='Omar Gouda',
    maintainer_email='oma2005r200@gmail.com',
    description='Three-node perception benchmark with obstacle, ArUco, scoring and live HTML.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'perception_reader_node = roar_perception_testing.perception_reader_node:main',
            'perception_tf_node = roar_perception_testing.perception_tf_node:main',
            'perception_benchmark_node = roar_perception_testing.perception_benchmark_node:main',
        ],
    },
)
