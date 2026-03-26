#!/usr/bin/env python3

# Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_frame_id',
            default_value='',
            description='frame_id for output topic'
        ),
        DeclareLaunchArgument(
            'start_angle_deg',
            default_value='-135.0',
            description='Sector start in degrees'
        ),
        DeclareLaunchArgument(
            'end_angle_deg',
            default_value='135.0',
            description='Sector end in degrees'
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value='/scan',
            description='Input LaserScan topic'
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/scan_filtered',
            description='Output LaserScan topic'
        ),
        DeclareLaunchArgument(
            'invert_sector',
            default_value='False',
            description='Invert sector (keep everything except specified)',
            choices=['True', 'False']
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation time',
            choices=['True', 'False']
        ),
        
        Node(
            package='lidar_filter',
            executable='lidar_filter',
            name='laser_filter',
            output='screen',
            parameters=[{
                'output_frame_id': ParameterValue(
                    LaunchConfiguration('output_frame_id'),
                    value_type=str
                ),
                'start_angle_deg': LaunchConfiguration('start_angle_deg'),
                'end_angle_deg': LaunchConfiguration('end_angle_deg'),
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'invert_sector': LaunchConfiguration('invert_sector'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
        )
    ])
