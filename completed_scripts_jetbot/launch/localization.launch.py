# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition



from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    localization_config_1_condition = PythonExpression([
        '"', LaunchConfiguration('config_choice'), '" == "1"'
    ])


    config_dir = get_package_share_directory('completed_scripts_jetbot')
    path_to_config_1 = os.path.join(config_dir, 'config', 'localization.yaml')


    localization_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'localization_launch.py'
                ])
            ]),
            launch_arguments={
                'slam_params_file': path_to_config_1
            }.items(),
            condition=IfCondition(localization_config_1_condition)
        )

    return [
        localization_launch,
    ]


def generate_launch_description():

    config_choice_arg = DeclareLaunchArgument(
        'config_choice',
        default_value='1',
        description='Select config file to use (1: localization.yaml )',
        choices=['1'],
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )


    
    return LaunchDescription([
        config_choice_arg,
        use_sim_time_arg,
        

        OpaqueFunction(function=launch_setup)
    ])