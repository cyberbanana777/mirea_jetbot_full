# Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

"""Веб-настройщик ПИД: http://<IP робота>:<port>."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080',
                              description='Порт веб-страницы'),
        DeclareLaunchArgument('host', default_value='0.0.0.0',
                              description='Адрес сервера (0.0.0.0 - все сети робота)'),
        DeclareLaunchArgument('step_test_max_duration', default_value='10.0',
                              description='Максимальная длительность ступенчатого теста, с'),
        Node(
            package='web_regulators_configurator',
            executable='web_configurator',
            name='web_regulators_configurator',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'step_test_max_duration': LaunchConfiguration('step_test_max_duration'),
            }],
        ),
    ])
