import os
from glob import glob

from setuptools import setup
package_name = 'completed_scripts_jetbot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "config", 'navigation'),
            glob("config/navigation/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob("rviz/*.rviz"),
        ),
        (
            os.path.join("share", package_name, "maps"),
            glob("maps/*.*"),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='banana-killer',
    maintainer_email='sashagrachev2005@gmail.com',
    description='This package include launch-files, what state the full system of jetbot from one click',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
