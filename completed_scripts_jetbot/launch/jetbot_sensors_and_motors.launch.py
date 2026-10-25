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
        'frame_id': "lidar_link",
        'angle_compensate': LaunchConfiguration('angle_compensate'),
        'scan_mode': LaunchConfiguration('scan_mode'),
    }

    # ========================================================================
    # Get paths to directories with launch-files and configs
    pkg1_launch_dir = os.path.join(
        get_package_share_directory('serial_bridge_cpp'),
        'launch'
    )
    pkg2_launch_dir = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch'
    )

    pkg3_launch_dir = os.path.join(
        get_package_share_directory('lidar_filter_cpp'),
        'launch'
    )

    camera_launch_dir = os.path.join(
        get_package_share_directory('realsense_d435i_configs'),
        'launch'
    )

    web_configurator_launch_dir = os.path.join(
        get_package_share_directory('web_regulators_configurator'),
        'launch'
    )

    # ========================================================================
    # Launches
    # Camera preset (rgb_usb3 / rgb_usb2 / rgb_depth / rgb_depth_pointcloud)
    # is selected by the camera_mode argument
    front_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            camera_launch_dir, 'd435i.launch.py'
            )
        ),
        launch_arguments={'mode': LaunchConfiguration('camera_mode')}.items(),
        condition=IfCondition(enable_front_camera_condition),
    )

    # Web UI for PID tuning: http://<robot IP>:<web_configurator_port>
    web_configurator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            web_configurator_launch_dir, 'web_configurator.launch.py'
            )
        ),
        launch_arguments={'port': LaunchConfiguration('web_configurator_port')}.items(),
        condition=IfCondition(PythonExpression([
            '"', LaunchConfiguration('enable_web_configurator'), '" == "True"'
        ])),
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
    
    lidar_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg3_launch_dir, 'filter.launch.py'
            )
        ),
        launch_arguments=sllidar_params.items(),
        condition=IfCondition(enable_lidar_condition),
    )

    # ========================================================================
    # System nodes
    # Attach the camera driver TF tree (camera_link -> camera_*_frame, factory
    # calibration) to the robot model: d435_link is the camera frame in the
    # URDF, both frames are at the origin of the depth module.
    transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'd435_link', 'camera_link'],
        condition=IfCondition(enable_front_camera_condition),
    )

    return [
        serial_bridge_launch,
        sllidar_launch,
        lidar_filter_launch,
        front_camera_launch,
        transform_node,
        web_configurator_launch,
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
        default_value='True',
        description='Enable/Disable front camera (Intel Realsense d435i)',
        choices=['True', 'False']
    )

    camera_preset_arg = DeclareLaunchArgument(
        'camera_mode',
        default_value='rgb_depth_pointcloud',
        description='Camera preset from realsense_d435i_configs '
                    '(used when enable_front_camera:=True)',
        choices=['rgb_usb3', 'rgb_usb2', 'rgb_depth', 'rgb_depth_pointcloud']
    )

    web_configurator_arg = DeclareLaunchArgument(
        'enable_web_configurator',
        default_value='False',
        description='Enable/Disable web UI for PID tuning '
                    '(web_regulators_configurator)',
        choices=['True', 'False']
    )

    web_configurator_port_arg = DeclareLaunchArgument(
        'web_configurator_port',
        default_value='8080',
        description='Port of the PID tuning web UI',
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


    
    return LaunchDescription([
        lidar_mode_arg,
        camera_mode_arg,
        camera_preset_arg,
        motors_mode_arg,
        web_configurator_arg,
        web_configurator_port_arg,

        lidar_serial_port_arg,
        angle_compensate_arg,
        scan_mode_arg,
        
        OpaqueFunction(function=launch_setup),
    ])