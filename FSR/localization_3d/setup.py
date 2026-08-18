from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'localization_3d'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        (os.path.join('share', package_name, 'maps'),
         glob(os.path.join('maps', '*.rviz'))),

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
            'localization_3D_manager = localization_3d.localization_3D_manager.py:main'
        ],
    },
)
