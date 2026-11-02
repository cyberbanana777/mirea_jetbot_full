# Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

# One C++ node instead of twist_to_command + serial_bridge_node +
# feedback_processor. Topics are the same as in serial_bridge_package:
#   in:  /cmd_vel, /low_level/serial/cmd (raw "$...#" strings)
#   out: /odom, TF odom->base_link, /sensors/wheel/{left,right}/speed,
#        /PIDcoeffs, /low_level/serial/feedback (raw frames)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', default_value='/dev/esp32',
            description='Serial port of the ESP32'),
        DeclareLaunchArgument(
            'baudrate', default_value='115200',
            description='Serial baudrate'),
        DeclareLaunchArgument(
            'publish_raw_feedback', default_value='true',
            description='Publish raw frames to serial/feedback '
                        '(needed for simple_PID_configurator)'),

        Node(
            package='serial_bridge_cpp',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            namespace='low_level',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'publish_raw_feedback': LaunchConfiguration('publish_raw_feedback'),
                'frame_id': 'odom',
                'child_frame_id': 'base_link',
                'publish_tf': True,
            }],
        ),
    ])
