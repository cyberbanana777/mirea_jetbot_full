# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Node-converter Twist to commands for ESP32
    twist_to_command = Node(
        package='serial_bridge_package',
        executable='twist_to_command',
        name='twist_to_command',
        namespace='low_level',
        output='screen',
        remappings=[
            ("/esp32_input", "serial/cmd"),
            ("/cmd_vel", "/cmd_vel"),
        ],
    )

    # Node-bridge between ROS and ESP32 thought serial
    bridge_node = Node(
        package='serial_bridge_package',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        namespace='low_level',
        output='screen',
        parameters=[{
            'serial_port': '/dev/esp32',
            'baudrate': 115200,
            'ros_in_topic': 'serial/cmd',
            'ros_out_topic': 'serial/feedback',
        }],
    )
    
    # Node processor feedback from ESP32
    feedback_node = Node(
        package='serial_bridge_package',
        executable='feedback_processor',
        name='feedback_processor',
        namespace='low_level',
        parameters=[{
            'frame_id': 'odom',
            'child_frame_id': 'base_link',
        }],
        output='screen',
        remappings=[
            ("/esp32_feedback", "serial/feedback"),
        ],
    )

    return LaunchDescription([
        twist_to_command,
        bridge_node,
        feedback_node

    ])