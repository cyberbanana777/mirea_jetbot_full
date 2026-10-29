import os
from glob import glob

from setuptools import setup

package_name = 'area_patrol'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='banana-killer',
    maintainer_email='sashagrachev2005@gmail.com',
    description='Endless Nav2 goals inside an area drawn with Publish Point in RViz',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'area_patrol = area_patrol.area_patrol_node:main',
        ],
    },
)
