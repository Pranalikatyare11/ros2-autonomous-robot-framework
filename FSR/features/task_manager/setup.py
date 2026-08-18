from glob import glob
from setuptools import find_packages, setup

package_name = 'task_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/tasks', glob('tasks/*.yaml')+ glob('tasks/*.json')),  # Add this line
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='panu',
    maintainer_email='pranalikatyare11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task_manager = task_manager.task_manager:main',
            'E_stop=task_manager.E_stop:main',
            'robot_status=task_manager.robot_status:main'
        ],
    },
)
