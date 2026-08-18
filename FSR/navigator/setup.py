from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
   
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
            
        ('share/' + package_name, ['package.xml']),

     
        ('share/' + package_name + '/launch', glob('launch/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='panuu',
    maintainer_email='pranalikatyare11@gmail.com',
    description='Navigation2 package for FSR project with planner switcher and BTs.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_switcher = navigator.planner_switcher:main',
            'navigator_manager =navigator.navigator_manager:main',
            'general_navigator = navigator.general_navigator:main',
            'state_manager = navigator.state_manager:main',
        ],
    },
)
