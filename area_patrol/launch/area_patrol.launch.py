import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('area_patrol'), 'config', 'area_patrol.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='random',
            description='random | circle | square', choices=['random', 'circle', 'square']),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='true — для симуляции Gazebo', choices=['true', 'false']),
        Node(
            package='area_patrol',
            executable='area_patrol',
            name='area_patrol',
            output='screen',
            emulate_tty=True,
            parameters=[
                config,
                {
                    'mode': LaunchConfiguration('mode'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                },
            ],
        ),
    ])
