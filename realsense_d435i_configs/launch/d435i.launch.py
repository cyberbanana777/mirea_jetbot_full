# Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

# Запуск камеры Intel RealSense D435i с одним из пресетов из config/.
# Пресет выбирается аргументом mode, по умолчанию rgb_depth_pointcloud.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODES = {
    'rgb_usb3': 'rgb_usb3.yaml',
    'rgb_usb2': 'rgb_usb2.yaml',
    'rgb_depth': 'rgb_depth.yaml',
    'rgb_depth_pointcloud': 'rgb_depth_pointcloud.yaml',
}


def _launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in MODES:
        raise RuntimeError(f"Unknown mode '{mode}', choose one of {list(MODES)}")

    name = LaunchConfiguration('camera_name').perform(context)
    ns = LaunchConfiguration('camera_namespace').perform(context)
    serial = LaunchConfiguration('serial_no').perform(context)
    extra = LaunchConfiguration('config_file').perform(context)

    cfg = extra or os.path.join(
        get_package_share_directory('realsense_d435i_configs'), 'config', MODES[mode])

    params = [cfg]
    if serial:
        # у обёртки серийник задаётся строкой с подчёркиванием: '_123456789'
        params.append({'serial_no': serial if serial.startswith('_') else '_' + serial})

    return [Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name=name,
        namespace=ns,
        parameters=params,
        output='screen',
        emulate_tty=True,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='rgb_depth_pointcloud',
                              description='rgb_usb3 | rgb_usb2 | rgb_depth | rgb_depth_pointcloud',
                              choices=list(MODES)),
        DeclareLaunchArgument('camera_name', default_value='camera'),
        DeclareLaunchArgument('camera_namespace', default_value='camera'),
        DeclareLaunchArgument('serial_no', default_value='',
                              description='Серийный номер, если камер несколько'),
        DeclareLaunchArgument('config_file', default_value='',
                              description='Свой yaml вместо пресета (необязательно)'),
        OpaqueFunction(function=_launch_setup),
    ])
