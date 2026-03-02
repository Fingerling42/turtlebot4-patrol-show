from glob import glob
from os.path import join

from setuptools import setup

package_name = 'turtlebot4-patrol-show'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=['tb4_patrol'],
    package_dir={'': 'turtlebot4-patrol-show'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ivan',
    maintainer_email='bermanivan42@gmail.com',
    description='TurtleBot4 show cycle demo: undock, lidar patrol, dock',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tb4_patrol = tb4_patrol:main',
        ],
    },
)
