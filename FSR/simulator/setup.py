from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'simulator'

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

        (os.path.join('share', package_name, '2D_description'),
         glob(os.path.join('2D_description', '*.xacro'))),

         (os.path.join('share', package_name, '3D_description'),
         glob(os.path.join('3D_description', '*.xacro'))),

        (os.path.join('share', package_name, 'worlds'),
         glob(os.path.join('worlds', '*.world'))), 

        (os.path.join('share', package_name, 'rviz'),
         glob(os.path.join('rviz', '*.rviz'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='katos',
    maintainer_email='katos@todo.todo',
    description='Differential Drive Robot with Gazebo and ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        
            'teleop = simulator.teleop:main',
            'general.simulator = simulator.general.simulator:main',
            'zone_recorder=simulator.zone_recorder:main',
            'zone_detector=simulator.zone_detector:main'
        ],
    },
)
