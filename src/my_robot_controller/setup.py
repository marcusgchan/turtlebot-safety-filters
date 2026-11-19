import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marcus',
    maintainer_email='marcus@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "test_node = my_robot_controller.first_node:main",
            "drive_forward = my_robot_controller.drive_forward:main",
            "safety_filter = my_robot_controller.safety_filter:main",
            "obstacle_detector = my_robot_controller.obstacle_detector:main",
            "debug_visualizer = my_robot_controller.debug_visualizer:main",
        ],
    },
)
