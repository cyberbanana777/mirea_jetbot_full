import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'web_regulators_configurator'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'static'), glob('static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='banana-killer',
    maintainer_email='sashagrachev2005@gmail.com',
    description='Web UI for tuning the wheel PID controllers of jetbot_mirea',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_configurator = web_regulators_configurator.node:main',
        ],
    },
)
