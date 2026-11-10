from setuptools import setup
from glob import glob
import os

package_name = 'pinky_mission'
setup(
    name=package_name, version='0.1.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='zxy', maintainer_email='zxy@example.com',
    description='Guarded Pinky waypoint missions for RViz2',
    license='Apache-2.0',
    entry_points={'console_scripts': ['mission_manager = pinky_mission.manager:main']},
)
