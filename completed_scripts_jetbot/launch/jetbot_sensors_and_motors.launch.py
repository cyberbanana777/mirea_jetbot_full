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

    # ========================================================================
    # Conditions
    
    enable_lidar_condition = PythonExpression([
        '"', LaunchConfiguration('enable_lidar'), '" == "True"'
    ])
    enable_front_camera_condition = PythonExpression([
        '"', LaunchConfiguration('enable_front_camera'), '" == "True"'
    ])
    enable_motors_condition = PythonExpression([
        '"', LaunchConfiguration('enable_motors'), '" == "True"'
    ])

    # ========================================================================
    # Param sets

    sllidar_params = {
        'serial_port': LaunchConfiguration('lidar_serial_port'),
        'frame_id': "rplidar_link",
        'angle_compensate': LaunchConfiguration('angle_compensate'),
        'scan_mode': LaunchConfiguration('scan_mode'),
    }

    # ========================================================================
    # Get paths to directories with launch-files and configs
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
    
    realsense_pkg_share = get_package_share_directory('realsense2_camera')
    realsense_launch_file = os.path.join(realsense_pkg_share, 'launch', 'rs_launch.py')
    

    # ========================================================================
    # Change env - configuration for realsense
    change_env = SetEnvironmentVariable(
        name='REALSENSE_CONFIG_FILE',
        value=config_file,
        condition=IfCondition(enable_front_camera_condition),
    )

    # ========================================================================
    # Launches
    front_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_file),
        condition=IfCondition(enable_front_camera_condition),
    )

    serial_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg1_launch_dir, 'serial_bringup.launch.py'
            )
        ),
        condition=IfCondition(enable_motors_condition),
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg2_launch_dir, 'sllidar_a2m8_launch.py'
            )
        ),
        launch_arguments=sllidar_params.items(),
        condition=IfCondition(enable_lidar_condition),
    )

    # ========================================================================
    # System nodes 
    transform_node =Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'realsense_link', 'camera_link'],
        condition=IfCondition(enable_front_camera_condition),
    )

    return [
        serial_bridge_launch,
        sllidar_launch,
        change_env,
        front_camera_launch,
        transform_node,
    ]


def generate_launch_description():
    # ========================================================================
    # Declare launch arguments

    lidar_mode_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='True',
        description='Enable/Disable lidar',
        choices=['True', 'False']
    )

    camera_mode_arg = DeclareLaunchArgument(
        'enable_front_camera',
        default_value='False',
        description='Enable/Disable front camera (Intel Realsense d435i)',
        choices=['True', 'False']
    )

    motors_mode_arg = DeclareLaunchArgument(
        'enable_motors',
        default_value='True',
        description='Enable/Disable motors',
        choices=['True', 'False']
    )

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
        lidar_mode_arg,
        camera_mode_arg,
        motors_mode_arg,

        lidar_serial_port_arg,
        angle_compensate_arg,
        scan_mode_arg,
        use_sim_time_arg,
        
        OpaqueFunction(function=launch_setup),
    ])