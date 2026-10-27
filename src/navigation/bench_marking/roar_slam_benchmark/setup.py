from setuptools import find_packages, setup

package_name = 'roar_slam_benchmark'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib', 'pandas', 'scipy', 'reportlab'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='SLAM Evaluator and Benchmarking Node',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_evaluator = roar_slam_benchmark.slam_evaluator:main'
        ],
    },
)
