# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os
from launch.conditions import IfCondition


def generate_launch_description():
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz_description',
        default_value='False',
        description='Запускает / не запускает RViz2.',
        choices=['True', 'False']
    )

    rviz_condition = PythonExpression([
        '"', LaunchConfiguration('launch_rviz_description'), '" == "True"'
    ])
    
    pkg_path = get_package_share_directory('jetbot_mirea_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'jetbot_mirea_simple_gazebo_real.urdf.xacro')
    robot_desc = Command(['xacro ', urdf_file])
    
    rviz_config_dir = os.path.join(pkg_path, 'rviz', 'real_set.rviz')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc,
                     'use_sim_time': False}]
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(rviz_condition),
    )
    
    return LaunchDescription([
        launch_rviz_arg,

        rviz_node,
        robot_state_publisher,
        joint_state_publisher        
    ])