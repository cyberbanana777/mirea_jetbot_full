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
    # Get value arg 'mode'
    # mode = LaunchConfiguration('mode').perform(context)

    # # Node parameters
    # serial_bridge_params = {
    #     'mode': mode,
    #     'target_topic': LaunchConfiguration('target_topic'),
    #     'max_joint_velocity': LaunchConfiguration('max_joint_velocity'),
    #     'target_action': LaunchConfiguration('target_action'),
    # }

    
    sllidar_params = {
        'serial_port': LaunchConfiguration('serial_port'),
        'frame_id': LaunchConfiguration('frame_id'),
        'angle_compensate': LaunchConfiguration('angle_compensate'),
        'scan_mode': LaunchConfiguration('scan_mode'),
    }
    
    # Get paths to directory with launch-files
    pkg1_launch_dir = os.path.join(
        get_package_share_directory('serial_bridge_package'),
        'launch'
    )
    pkg2_launch_dir = os.path.join(
        get_package_share_directory('jetbot_mirea_description'),
        'launch'
    )
    pkg3_launch_dir = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch'
    )

    config_dir = get_package_share_directory('completed_scripts_jetbot')
    config_file = os.path.join(config_dir, 'config', 'realsense_config.yaml')
    
    # Получаем путь к системному launch-файлу realsense2_camera
    realsense_pkg_share = get_package_share_directory('realsense2_camera')
    realsense_launch_file = os.path.join(realsense_pkg_share, 'launch', 'rs_launch.py')
    
    rviz_config_dir = os.path.join(get_package_share_directory('completed_scripts_jetbot'), 'rviz', 'full.rviz')
    rviz_condition = PythonExpression([
        '"', LaunchConfiguration('launch_rviz'), '" == "True"'
    ])

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
        # launch_arguments=serial_bridge_params.items()
    )

    jetbos_mirea_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg2_launch_dir, 'jetbot_mirea_rviz.launch.py'
            )
        ),
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg3_launch_dir, 'sllidar_a2m8_launch.py'
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(rviz_condition),
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'config', 'mapper_params_online_async.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/odom', '/odom'),
        ]
    )

    return [
        serial_bridge_launch,
        jetbos_mirea_description_launch,
        sllidar_launch,
        change_env,
        camera_launch,
        transform_node,
        # rviz_node,
        # slam_node,

    ]


def generate_launch_description():
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='True',
        description='Запускает / не запускает RViz2.',
        choices=['True', 'False']
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/rplidar',
        description='Specifying usb port to connected lidar',
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='rplidar_link',
        description='Specifying frame_id of lidar',
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

    # target_topic_arg = DeclareLaunchArgument(
    #     'target_topic',
    #     default_value='arm_sdk',
    #     description='Topic for control commands.',
    #     choices=['arm_sdk', 'lowcmd'],
    # )

    # max_joint_velocity_arg = DeclareLaunchArgument(
    #     'max_joint_velocity',
    #     default_value='4.0',
    #     description='Maximum joint velocity.',
    # )

    # target_action_arg = DeclareLaunchArgument(
    #     'target_action',
    #     default_value='teleoperation',
    #     description='Target action for control commands.',
    #     choices=['other', 'teleoperation'],
    # )

    # metrics_arg = DeclareLaunchArgument(
    #     "enable_metrics",
    #     default_value="False",
    #     description="Enable metrics publishing.",
    #     choices=['True', 'False'],
    # )

    # joints_to_check_arg = DeclareLaunchArgument(
    #     "joints_to_check",
    #     default_value="12, 13, 14, 15, 31, 33",
    #     description="Joints to check. Valided joints must " \
    #     "be in range [0, 33], excluding 9.",
    # )

    # ip_arg = DeclareLaunchArgument(
    #     "ip",
    #     default_value="192.168.123.162",
    #     description="Ip address for reading data from the UKT",
    # )


    
    return LaunchDescription([
        launch_rviz_arg,
        serial_port_arg,
        frame_id_arg,
        angle_compensate_arg,
        scan_mode_arg,
        use_sim_time_arg,
        
        # target_topic_arg,
        # max_joint_velocity_arg,
        # target_action_arg,
        # metrics

        OpaqueFunction(function=launch_setup)
    ])