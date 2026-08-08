import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'roar_rover_tf_publisher'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@email.com',
    description='Minimal node that publishes only the TF tree of the ROAR rover (rover_clean).',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_tf_tree_publisher = roar_rover_tf_publisher.tf_tree_publisher:main',
        ],
    },
)
