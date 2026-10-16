import os
from glob import glob 
from setuptools import setup

package_name = 'serial_bridge_package'

setup(
    name=package_name,
    version='1.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'rviz'), 
            glob('rviz/*.rviz')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='banana-killer',
    maintainer_email='sashagrachev2005@gmail.com',
    description='This package include node for provide data transfer from ' \
        'microcontroller to computer and back thought Serial port.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge_node = serial_bridge_package.serial_bridge_node:main',
            'twist_to_command = serial_bridge_package.twist_to_command:main',
            'feedback_processor = serial_bridge_package.feedback_processor:main',
        ],
    },
)
