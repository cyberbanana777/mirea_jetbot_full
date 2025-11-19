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
    localization_config_2_condition = PythonExpression([
        '"', LaunchConfiguration('config_choice'), '" == "2"'
    ])


    config_dir = get_package_share_directory('completed_scripts_jetbot')
    path_to_config_1 = os.path.join(config_dir, 'config', 'mapper_params_online_async.yaml')
    path_to_config_2 = os.path.join(config_dir, 'config', 'mapping.yaml')


    localization_launch_1 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_sync_launch.py'
                ])
            ]),
            launch_arguments={
                'slam_params_file': path_to_config_1
            }.items(),
            condition=IfCondition(localization_config_1_condition)
        )
    

    localization_launch_2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_sync_launch.py'
                ])
            ]),
            launch_arguments={
                'slam_params_file': path_to_config_2
            }.items(),
            condition=IfCondition(localization_config_2_condition)
        )

    return [
        localization_launch_1,
        localization_launch_2,
    ]


def generate_launch_description():

    config_choice_arg = DeclareLaunchArgument(
        'config_choice',
        default_value='1',
        description='Select config file to use (1: mapper_params_online_async.yaml   2: mapping.yaml)',
        choices=['1', '2'],
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