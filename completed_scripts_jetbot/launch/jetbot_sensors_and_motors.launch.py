# Copyright (c) 2025 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition



def launch_setup(context, *args, **kwargs):
    
    sllidar_params = {
        'serial_port': LaunchConfiguration('lidar_serial_port'),
        'frame_id': "rplidar_link",
        'angle_compensate': LaunchConfiguration('angle_compensate'),
        'scan_mode': LaunchConfiguration('scan_mode'),
    }

    
    # Get paths to directory with launch-files
    pkg1_launch_dir = os.path.join(
        get_package_share_directory('serial_bridge_package'),
        'launch'
    )
    pkg2_launch_dir = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch'
    )

    config_dir = get_package_share_directory('completed_scripts_jetbot')
    config_file = os.path.join(config_dir, 'config', 'realsense_config.yaml')
    
    # Получаем путь к системному launch-файлу realsense2_camera
    realsense_pkg_share = get_package_share_directory('realsense2_camera')
    realsense_launch_file = os.path.join(realsense_pkg_share, 'launch', 'rs_launch.py')
    

    change_env = SetEnvironmentVariable(
        name='REALSENSE_CONFIG_FILE',
        value=config_file
    )
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file)
    )

    serial_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg1_launch_dir, 'serial_bringup.launch.py'
            )
        ),
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg2_launch_dir, 'sllidar_a2m8_launch.py'
            )
        ),
        launch_arguments=sllidar_params.items()
    )

    transform_node =Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'realsense_link', 'camera_link']
    )

    return [
        serial_bridge_launch,
        sllidar_launch,
        change_env,
        camera_launch,
        transform_node,

    ]


def generate_launch_description():


    lidar_serial_port_arg = DeclareLaunchArgument(
        'lidar_serial_port',
        default_value='/dev/rplidar',
        description='Specifying usb port to connected lidar',
    )

    angle_compensate_arg = DeclareLaunchArgument(
        'angle_compensate',
        default_value='true',
        description='Specifying whether or not to enable angle_compensate of scan data',
        choices=['true', 'false']
    )
    
    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Standard',
        description='Specifying scan mode of lidar Standard: max_distance: 12.0 m, Point number: 2.0K Express: max_distance: 12.0 m, Point number: 4.0K \nBoost: max_distance: 12.0 m, Point number: 8.0K',
        choices=['Standard', 'Express', 'Boost'],
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )



    
    return LaunchDescription([
        lidar_serial_port_arg,

        angle_compensate_arg,
        scan_mode_arg,
        use_sim_time_arg,
        
        OpaqueFunction(function=launch_setup)
    ])