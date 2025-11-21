from setuptools import setup

package_name = 'trajectory_maker'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='banana-killer',
    maintainer_email='sashagrachev2005@gmail.com',
    description="The package consist of some nodes for creating trajectory for jetbot's moving.",
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'start_move_action = trajectory_maker.start_move_action:main',
            'start_move_topic = trajectory_maker.start_move_topic:main',

        ],
    },
)
