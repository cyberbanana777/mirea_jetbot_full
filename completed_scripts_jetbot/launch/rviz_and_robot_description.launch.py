# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node




def launch_setup(context, *args, **kwargs):
  
    pkg1_launch_dir = os.path.join(
        get_package_share_directory('jetbot_mirea_description'),
        'launch'
    )

    rviz_config_dir = os.path.join(get_package_share_directory('completed_scripts_jetbot'), 'rviz', 'full.rviz')

    jetbos_mirea_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg1_launch_dir, 'jetbot_mirea_rviz.launch.py'
            )
        ),
    )

    transform_node =Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'realsense_link', 'camera_link']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}],
    )

    return [
        jetbos_mirea_description_launch,
        transform_node,
        rviz_node,

    ]


def generate_launch_description():
    
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])