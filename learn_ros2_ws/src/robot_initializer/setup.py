import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_initializer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'),
        glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'rviz'),
        glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'models'),
        glob(os.path.join('models', '*.sdf')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kangzhao',
    maintainer_email='kangzhao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_initializer_node = robot_initializer.robot_initializer_node:main',
            'detected_object_processer_node = robot_initializer.detected_object_processer_node:main',
            'robot_explorer_node = robot_initializer.robot_explorer_node:main'
        ],
    },
)
