# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def launch_setup(context, *args, **kwargs):

    acml_config_1_condition = PythonExpression([
        '"', LaunchConfiguration('config_choice'), '" == "1"'
    ])    
    acml_config_2_condition = PythonExpression([
        '"', LaunchConfiguration('config_choice'), '" == "2"'
    ])


    path_to_pkg = get_package_share_directory('completed_scripts_jetbot')

    acml_config_yaml_real = os.path.join(path_to_pkg, 'config', 'amcl_params_real.yaml')
    acml_config_yaml_sim = os.path.join(path_to_pkg, 'config', 'amcl_params_sim.yaml')
    map_file = os.path.join(path_to_pkg, 'maps', 'map.yaml')
    # map_file = os.path.join(path_to_pkg, 'maps', 'G210_with_boxes_map.yaml')

    acml_node_real = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[acml_config_yaml_real],
        condition=IfCondition(acml_config_1_condition),
    )
    
    acml_node_sim = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[acml_config_yaml_sim],
        condition=IfCondition(acml_config_2_condition),
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'autostart': True},
            {
                'node_names': [
                    'map_server',
                    'amcl',
                    ]
            }
        ]
    )
        
    map_loader_real = TimerAction(
        period=2.0,  # Задержка в секундах
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'use_sim_time': False},
                            {'yaml_filename': map_file}],
                condition=IfCondition(acml_config_1_condition),
            ),
        ]
    )

    map_loader_sim = TimerAction(
        period=2.0,  # Задержка в секундах
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'use_sim_time': True},
                            {'yaml_filename': map_file}],
                condition=IfCondition(acml_config_2_condition),
                
            ),
        ]
    )

    return [
        acml_node_real,
        acml_node_sim,
        lifecycle_manager,
        map_loader_real,
        map_loader_sim,
    ]

def generate_launch_description():

    config_choice_arg = DeclareLaunchArgument(
        'config_choice',
        default_value='1',
        description='Select config file to use (1: mapper_params_real.yaml   2: mapper_params_sim.yaml)',
        choices=['1', '2'],
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
        choices=['true', 'false'],
    )

    return LaunchDescription([

        config_choice_arg,
        use_sim_time_arg,

        OpaqueFunction(function=launch_setup)
    ])