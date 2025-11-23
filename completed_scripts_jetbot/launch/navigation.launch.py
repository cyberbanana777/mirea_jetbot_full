# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():

    path_to_pkg = get_package_share_directory('completed_scripts_jetbot')

    controller_yaml = os.path.join(path_to_pkg, 'config', 'navigation', 'controller.yaml')
    default_bt_xml_path = os.path.join(path_to_pkg, 'config', 'navigation', 'behavior.xml')
    planner_yaml = os.path.join(path_to_pkg, 'config', 'navigation', 'planner_server.yaml')
    bt_navigator_yaml = os.path.join(path_to_pkg, 'config', 'navigation', 'bt_navigator.yaml')
    nav2_yaml = os.path.join(path_to_pkg, 'config', 'navigation', 'amcl_config.yaml')
    map_file = os.path.join(path_to_pkg, 'maps', 'map.yaml')
    rviz_config_file_path = os.path.join(path_to_pkg, 'rviz', 'pathplanning.rviz')
    waypoint_follower_yaml = os.path.join(path_to_pkg, 'config', 'navigation','waypoint_follower.yaml')


    return LaunchDescription([
        
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),
        # Node(
        #     package='nav2_controller',
        #     executable='controller_server',
        #     name='controller_server',
        #     output='screen',
        #     parameters=[controller_yaml]),

        # Node(
        #     package='nav2_planner',
        #     executable='planner_server',
        #     name='planner_server',
        #     output='screen',
        #     parameters=[planner_yaml]),

        # Node(
        #     package='nav2_bt_navigator',
        #     executable='bt_navigator',
        #     name='bt_navigator',
        #     output='screen',
        #     parameters=[bt_navigator_yaml, {'default_bt_xml_filename': default_bt_xml_path}]),

        # Node(
        #     package='nav2_waypoint_follower',
        #     executable='waypoint_follower',
        #     name='waypoint_follower',
        #     output='screen',
        #     parameters=[waypoint_follower_yaml]),
        

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        #'controller_server',
                                        #'planner_server',
                                        #'bt_navigator',
                                        #'waypoint_follower'
                                        ]}]),
        TimerAction(
            period=2.0,  # Задержка в секундах
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[{'use_sim_time': False},
                                {'yaml_filename': map_file}]
                ),
            ]
        )
    ])